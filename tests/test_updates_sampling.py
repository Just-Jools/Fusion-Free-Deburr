"""Regressions for sketch replacement and normal-sensitive sampling."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

from test_safety import fusion, dm, adsk


class UpdateTests(unittest.TestCase):
    def test_failed_updates_restore_operation_and_remove_replacement(self):
        for failure in ('curves', 'settings', 'delete', None):
            with self.subTest(failure=failure):
                new = MagicMock()

                class OldSketch:
                    def __init__(self):
                        self._name = 'Deburr'
                        self.deleteMe = MagicMock()

                    @property
                    def name(self):
                        return self._name

                    @name.setter
                    def name(self, name):
                        # Fusion adds a suffix if the replacement still owns this name.
                        occupied = new.name == name and not new.deleteMe.called
                        self._name = name + ' (1)' if occupied else name

                old = OldSketch()
                old.deleteMe.return_value = failure != 'delete'
                params = {n: NS(expression='original') for n in
                          ('compensation', 'compensationType', 'axialOffset', 'radialOffset')}
                op = MagicMock(name='operation')
                op.name, op.tool = 'Deburr', 'old tool'
                op.parameters.itemByName.side_effect = params.get
                attr = NS(value='original settings')
                op.attributes.itemByName.return_value = attr
                op.attributes.add.side_effect = lambda group, name, data: setattr(attr, 'value', data)
                value = MagicMock()
                value.getCurveSelections.return_value = ['old curves']
                selected = ['old curves']
                value.applyCurveSelections.side_effect = lambda curves: selected.__setitem__(slice(None), curves)

                def update(op, curves):
                    params['axialOffset'].expression = '0 mm'
                    selected[:] = ['new curves']
                    if failure == 'curves':
                        raise RuntimeError('curve application failed')

                def save(*args):
                    attr.value = 'new settings'
                    if failure == 'settings':
                        raise RuntimeError('settings save failed')

                with patch.object(fusion, 'load_settings', return_value={'sketch': 'old token'}), \
                     patch.object(fusion, 'find_by_token', return_value=old), \
                     patch.object(fusion, '_curves_parameter', return_value=value), \
                     patch.object(fusion, 'build_sketch', return_value=(new, [['new curves']])), \
                     patch.object(fusion, 'update_trace_curves', side_effect=update), \
                     patch.object(fusion, 'save_settings', side_effect=save):
                    if failure:
                        with self.assertRaises(Exception):
                            fusion.update_operation(None, op, [], [], .1, 'new tool')
                        self.assertEqual(op.tool, 'old tool')
                        self.assertEqual(selected, ['old curves'])
                        self.assertEqual(attr.value, 'original settings')
                        self.assertEqual(params['axialOffset'].expression, 'original')
                        self.assertEqual(old.name, 'Deburr')
                        new.deleteMe.assert_called_once()
                        if failure != 'delete':
                            old.deleteMe.assert_not_called()
                    else:
                        self.assertIs(fusion.update_operation(None, op, [], [], .1, 'new tool'), new)
                        self.assertEqual(op.tool, 'new tool')
                        old.deleteMe.assert_called_once()
                        new.deleteMe.assert_not_called()

    def test_partial_sketch_is_removed_when_building_fails(self):
        design = MagicMock()
        sketch = design.rootComponent.sketches.add.return_value
        sketch.sketchCurves.sketchFixedSplines.addByNurbsCurve.side_effect = RuntimeError('spline failure')
        with patch.object(fusion.dc, 'chain_polylines', return_value=[[(0, 0, 0), (1, 0, 0)]]):
            with self.assertRaisesRegex(RuntimeError, 'spline failure'):
                fusion.build_sketch(design, [object()], 'Deburr')
        sketch.deleteMe.assert_called_once()

    def test_failed_trace_creation_removes_operation(self):
        setup = MagicMock()
        op = setup.operations.add.return_value
        with patch.object(fusion, '_trace_settings', side_effect=RuntimeError('settings failure')):
            with self.assertRaises(RuntimeError):
                fusion.create_trace(setup, [], 'Deburr')
        op.deleteMe.assert_called_once()

    def test_rollback_failure_keeps_geometry_that_may_be_referenced(self):
        op, old, new, value = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        old.name = 'Deburr'
        value.applyCurveSelections.side_effect = RuntimeError('restore failure')
        with patch.object(fusion, 'load_settings', return_value={'sketch': 'old'}), \
             patch.object(fusion, 'find_by_token', return_value=old), \
             patch.object(fusion, '_curves_parameter', return_value=value), \
             patch.object(fusion, 'build_sketch', return_value=(new, [])), \
             patch.object(fusion, 'update_trace_curves', side_effect=RuntimeError('update failure')):
            with self.assertRaisesRegex(dm.DeburrError, 'both sketches retained'):
                fusion.update_operation(None, op, [], [], .1)
        old.deleteMe.assert_not_called()
        new.deleteMe.assert_not_called()


class SamplingTests(unittest.TestCase):
    def edge(self):
        ev = MagicMock()
        ev.getParameterExtents.return_value = (True, 0, .01)
        ev.getLengthAtParameter.side_effect = lambda a, b: (True, b - a)
        ev.getPointAtParameter.side_effect = lambda t: (True, NS(x=t, y=0, z=0))
        ev.getParameterAtPoint.side_effect = lambda p: (True, p[0])
        edge = NS(evaluator=ev, faces=MagicMock())
        edge.faces.item.side_effect = lambda i: i
        return edge

    def normal(self, face, p):
        angle = math.pi * p[0] / .02
        return (0, 0, 1) if face == 0 else (math.cos(angle), math.sin(angle), 0)

    def test_straight_edge_with_changing_normals_refines_tool_tip(self):
        edge = self.edge()
        points = [(0, 0, 0), (.01, 0, 0)]
        with patch.object(fusion, '_normal', side_effect=self.normal):
            samples = fusion._refine_samples(edge, points, .3, .025, (0, 0, 1))
        self.assertGreater(len(samples), 2)
        def tip(p):
            return dm.tool_tip(p, self.normal(0, p), self.normal(1, p), (0, 0, 1), .3, .025)
        for a, b in zip(samples, samples[1:]):
            for k in range(11):
                t = k / 10
                p = dm.add(a, dm.scale(dm.sub(b, a), t))
                chord = dm.add(tip(a), dm.scale(dm.sub(tip(b), tip(a)), t))
                self.assertLessEqual(fusion.dc.dist(tip(p), chord), fusion.PATH_TOL)

    def test_constant_normals_on_short_straight_edge_need_only_two_points(self):
        edge = self.edge()
        with patch.object(fusion, '_normal', side_effect=lambda face, p: (0, 0, 1) if face == 0 else (1, 0, 0)):
            samples = fusion._refine_samples(edge, [(0, 0, 0), (.01, 0, 0)], .3, .025, (0, 0, 1))
        self.assertEqual(len(samples), 2)

    def test_unresolved_normal_change_is_rejected_at_refinement_limit(self):
        edge = self.edge()
        def normal(face, p):
            if face == 0:
                return (0, 0, 1)
            return (1, 0, 0) if p[0] == 0 else (0, 1, 0)
        with patch.object(fusion, '_normal', side_effect=normal):
            with self.assertRaisesRegex(dm.DeburrError, 'sampling tolerance'):
                fusion._refine_samples(edge, [(0, 0, 0), (.01, 0, 0)], .3, .025, (0, 0, 1))
        self.assertLessEqual(edge.evaluator.getPointAtParameter.call_count, 13)

    def test_closed_seam_uses_parameter_extent_instead_of_ambiguous_point_lookup(self):
        edge = self.edge()
        ev = edge.evaluator
        radius = .002
        ev.getParameterExtents.return_value = (True, 0, 2 * math.pi)
        ev.getLengthAtParameter.side_effect = lambda a, b: (True, radius * (b - a))
        ev.getPointAtParameter.side_effect = lambda t: (True, NS(x=radius * math.cos(t), y=radius * math.sin(t), z=0))
        ev.getParameterAtPoint.side_effect = lambda p: (True, math.atan2(p[1], p[0]) % (2 * math.pi))
        points = [(radius, 0, 0), (0, radius, 0), (-radius, 0, 0), (0, -radius, 0), (radius, 0, 0)]
        with patch.object(fusion, '_normal', side_effect=lambda face, p: (0, 0, 1) if face == 0 else (1, 0, 0)), \
             patch.object(adsk.core.Point3D, 'create', side_effect=lambda *p: p):
            fusion._refine_samples(edge, points, .3, .025, (0, 0, 1))
        self.assertAlmostEqual(ev.getPointAtParameter.call_args.args[0], 7 * math.pi / 4)

    def test_small_closed_edges_fail_gracefully_before_indexing(self):
        edge = MagicMock()
        edge.faces.count = 2
        for points in ([], [(0, 0, 0)], [(0, 0, 0)] * 2,
                       [(0, 0, 0), (1, 0, 0), (0, 0, 0)], [(0, 0, 0)] * 4):
            with self.subTest(points=points), \
                 patch.object(fusion, '_is_closed', return_value=True), \
                 patch.object(fusion, '_sample_edge', return_value=points):
                with self.assertRaisesRegex(dm.DeburrError, 'too few'):
                    fusion.edge_segment(edge, .3, .025, (0, 0, 1), lambda f: 0)
