"""Undercut linking, side access, and clearance regressions."""
import importlib
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

from test_safety import adsk, dc, dm, fusion

with patch.dict('sys.modules', {'adsk': adsk, 'adsk.core': adsk.core,
                               'adsk.fusion': adsk.fusion, 'adsk.cam': adsk.cam}):
    lf = importlib.import_module('safety_fusion.lollipop_fusion')


class LollipopTests(unittest.TestCase):
    def test_rotated_cylinder_bounds_use_radial_projection(self):
        component = math.sqrt(.5)
        lo, hi = lf._cylinder_bounds((0, 0, 0), (1, 0, 1), 2,
                                     (component, 0, component))
        self.assertAlmostEqual(lo[0], -math.sqrt(2))
        self.assertAlmostEqual(lo[1], -2)
        self.assertAlmostEqual(lo[2], -math.sqrt(2))
        self.assertAlmostEqual(hi[0], 1 + math.sqrt(2))
        self.assertAlmostEqual(hi[2], 1 + math.sqrt(2))

    def test_holder_above_part_skips_intersection_without_ignoring_head(self):
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        body = NS(pointContainment=lambda p: outside,
                  boundingBox=NS(minPoint=NS(x=-5, y=-4, z=0),
                                 maxPoint=NS(x=6, y=4, z=4.7)))
        cutter = NS(radius=.4, noncutting_sections=((5, 8, 3.2),))
        check = lf.Clearance(cutter, (0, 0, 1), [body], [], .02)
        check.measure = MagicMock()
        check.measure.measureMinimumDistance.return_value.value = .4
        check.temp = MagicMock()
        self.assertIsNone(check.point((0, 0, 0), .001))
        check.temp.createCylinderOrCone.assert_not_called()
        check.measure.measureMinimumDistance.assert_called_once()

    def test_cam_proxy_resolves_to_design_body_by_identity_and_placement(self):
        def box(x=0):
            return NS(minPoint=NS(x=x, y=0, z=0), maxPoint=NS(x=x+1, y=1, z=1))
        body = NS(isSolid=True, nativeObject=None, boundingBox=box(), assemblyContext=None)
        proxy = NS(isSolid=True, nativeObject=body, boundingBox=box(), assemblyContext=None)
        design = NS(rootComponent=NS(bRepBodies=[body], allOccurrences=[]))
        setup = NS(models=[proxy], fixtures=[])
        with patch.object(adsk.fusion.BRepBody, 'cast', side_effect=lambda e: e), \
                patch.object(adsk.fusion.Occurrence, 'cast', return_value=None):
            models, fixtures = lf.setup_obstacles(setup, design)
            self.assertIs(models[0], body)
            self.assertEqual(fixtures, [])
            proxy.boundingBox = box(.1)
            with self.assertRaisesRegex(dm.DeburrError, 'repositioned manufacturing'):
                lf.setup_obstacles(setup, design)

    def test_selected_cam_edges_resolve_into_design_and_keep_native_picks(self):
        box = NS(minPoint=NS(x=0, y=0, z=0), maxPoint=NS(x=1, y=1, z=1))
        body = NS(nativeObject=None, assemblyContext=None, boundingBox=box)
        proxy = NS(nativeObject=body, assemblyContext=None, boundingBox=box)
        native = NS(body=body, nativeObject=None)
        picked = NS(body=proxy, nativeObject=native)
        resolved = lf.setup_edges([picked, native], [body])
        self.assertIs(resolved[0], native)
        self.assertIs(resolved[1], native)
        with patch.object(lf, 'Clearance'), patch.object(lf, '_directions', return_value=[]), \
                patch.object(lf.df, 'edge_segment', return_value='segment') as sample, \
                patch.object(lf.dc, 'order_chains', return_value=[]):
            chains, used, notes = lf.build_chains([picked], NS(radius=.4), .02,
                                                 (0, 0, 1), [body])
        self.assertIs(sample.call_args.args[0], native)
        self.assertIs(used[0], native)

    def test_selected_cam_edge_retains_design_occurrence_placement(self):
        box = NS(minPoint=NS(x=2, y=0, z=0), maxPoint=NS(x=3, y=1, z=1))
        matrix = [1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        context = NS(transform2=NS(asArray=lambda: matrix))
        native_body = NS(nativeObject=None, assemblyContext=None)
        body = NS(nativeObject=native_body, assemblyContext=context, boundingBox=box)
        proxy = NS(nativeObject=native_body, assemblyContext=NS(
            transform2=NS(asArray=lambda: list(matrix))), boundingBox=box)
        mapped = NS(body=body)
        native = NS(createForAssemblyContext=MagicMock(return_value=mapped))
        self.assertIs(lf.setup_edges([NS(body=proxy, nativeObject=native)], [body])[0], mapped)
        native.createForAssemblyContext.assert_called_once_with(context)

    def test_selected_edge_rejects_wrong_model_translation_and_symmetric_rotation(self):
        box = NS(minPoint=NS(x=-1, y=-1, z=-1), maxPoint=NS(x=1, y=1, z=1))
        body = NS(nativeObject=None, assemblyContext=None, boundingBox=box)
        wrong = NS(nativeObject=None, assemblyContext=None, boundingBox=box, other=True)
        translated = [1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        rotated = [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        proxies = [wrong] + [NS(nativeObject=body, boundingBox=box, assemblyContext=NS(
            transform2=NS(asArray=lambda m=m: m))) for m in (translated, rotated)]
        for proxy in proxies:
            with self.subTest(proxy=proxy), self.assertRaisesRegex(dm.DeburrError, 'must belong'):
                lf.setup_edges([NS(body=proxy)], [body])

    def test_unresolvable_edge_proxy_is_rejected(self):
        box = NS(minPoint=NS(x=0, y=0, z=0), maxPoint=NS(x=1, y=1, z=1))
        context = NS(transform2=NS(asArray=lambda: [1, 0, 0, 0, 0, 1, 0, 0,
                                                   0, 0, 1, 0, 0, 0, 0, 1]))
        native_body = NS(nativeObject=None)
        body = NS(nativeObject=native_body, boundingBox=box, assemblyContext=context)
        proxy = NS(nativeObject=native_body, boundingBox=box, assemblyContext=None)
        native = NS(createForAssemblyContext=lambda c: None)
        with self.assertRaisesRegex(dm.DeburrError, 'could not resolve'):
            lf.setup_edges([NS(body=proxy, nativeObject=native)], [body])

    def test_trace_preserves_approach_and_withdrawal_geometry(self):
        params = {name: NS(expression='unsafe') for name, _ in fusion.TRACE_EXPRESSIONS}
        op = NS(parameters=NS(itemByName=params.get))
        fusion._trace_settings(op)
        for name in ('doLeadIn', 'doLeadOut', 'smoothingFilter', 'orderByDepth'):
            self.assertEqual(params[name].expression, 'false')
        self.assertEqual(params['preserveOrder'].expression, 'true')
        self.assertEqual(params['retractionPolicy'].expression, "'full'")

    def test_upper_hemisphere_corner_roll_uses_lollipop_limit(self):
        a = dc.End((0, 0, 0), {'shared': (0, 0, -1), 'a': (1, 0, 0)})
        b = dc.End((0, 0, 0), {'shared': (0, 0, -1), 'b': (0, 1, 0)})
        with self.assertRaises(dm.DeburrError):
            dc.corner_roll(a, b, (0, 0, 1), .4, .02)
        limit = dm.lollipop_contact_height(.4, .2, .8)
        points = dc.corner_roll(a, b, (0, 0, 1), .4, .02, limit)
        self.assertGreater(len(points), 10)
        self.assertAlmostEqual(points[0][2], points[-1][2])

    def test_lollipop_does_not_fall_back_to_unbounded_vertex_crossing(self):
        cutter = NS(max_contact_height=.3)
        notes = []
        with patch.object(dc, 'corner_roll', side_effect=dm.DeburrError('neck contact')), \
                patch.object(dc, 'vertex_crossing') as crossing:
            result = dc._corner(0, None, None, (0, 0, 0), (1, 0, 0),
                                (0, 0, 1), .4, .02, None, notes, cutter)
        self.assertIsNone(result)
        crossing.assert_not_called()
        self.assertIn('paths split', notes[0])

    def test_closed_cut_gets_distinct_exterior_endpoints(self):
        box = NS(minPoint=NS(x=-1, y=-1, z=0), maxPoint=NS(x=1, y=1, z=3))
        check = NS(bodies=[NS(boundingBox=box)], fixtures=[],
                   cutter=NS(radius=.4, noncutting_sections=((.7, 2, .2),)),
                   path=MagicMock(return_value=None))
        points = [(0, -1.3, 1), (.2, -1.3, 1), (0, -1.3, 1)]
        result = lf.add_access(points, [(0, -1, 0)], check)
        self.assertEqual(result[1:-1], points)
        self.assertNotEqual(result[0], result[-1])
        self.assertLess(result[0][1], -1.4)
        self.assertLess(result[-1][1], result[0][1])

    def test_blocked_access_is_rejected(self):
        check = NS(bodies=[NS(boundingBox=NS(minPoint=NS(x=0, y=0, z=0),
                                             maxPoint=NS(x=1, y=1, z=1)))],
                   fixtures=[], cutter=NS(radius=.4, noncutting_sections=()),
                   path=MagicMock(return_value='shaft collision'))
        with self.assertRaisesRegex(dm.DeburrError, 'no clear lollipop approach'):
            lf.add_access([(0, 0, 0), (1, 0, 0)], [(0, -1, 0)], check)

    def test_clearance_samples_between_endpoints(self):
        check = lf.Clearance(NS(), (0, 0, 1), [], [], .02)
        check.point = MagicMock(side_effect=lambda p, margin: 'collision' if .49 < p[0] < .51 else None)
        self.assertEqual(check.path([(0, 0, 0), (1, 0, 0)]), 'collision')
        self.assertGreater(check.point.call_count, 40)
        for call in check.point.call_args_list:
            self.assertGreaterEqual(call.args[1], lf.CLEARANCE_MARGIN)

    def test_close_clearance_refines_instead_of_rejecting_small_breaks(self):
        check = lf.Clearance(NS(), (0, 0, 1), [], [], .001)
        check.point = MagicMock(side_effect=lambda p, margin: 'coarse envelope' if margin > .0012 else None)
        self.assertIsNone(check.path([(0, 0, 0), (.01, 0, 0)]))
        self.assertGreater(check.point.call_count, 10)

    def test_neck_collision_and_failed_intersection_are_rejected(self):
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        body = NS(pointContainment=lambda p: outside)
        cutter = NS(radius=.4, noncutting_sections=((.75, 2, .2),))
        for success, faces, expected in ((True, 1, 'neck, shaft'), (True, 0, None),
                                          (False, 0, 'could not check')):
            check = lf.Clearance(cutter, (0, 0, 1), [body], [], .02)
            check.measure = MagicMock()
            check.measure.measureMinimumDistance.return_value.value = .4
            check.temp = MagicMock()
            check.temp.booleanOperation.return_value = success
            check.temp.createCylinderOrCone.return_value.faces.count = faces
            with patch.object(lf, '_overlap', return_value=True), \
                    patch.object(adsk.core.Point3D, 'create', side_effect=lambda *p: p):
                if not success:
                    with self.assertRaisesRegex(dm.DeburrError, expected):
                        check.point((0, 0, 0), .001)
                else:
                    result = check.point((0, 0, 0), .001)
                    if expected:
                        self.assertIn(expected, result)
                    else:
                        self.assertIsNone(result)

    def test_fixtures_have_no_edge_break_allowance(self):
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        body = NS(pointContainment=lambda p: outside)
        for fixtures, expected in (([], None), ([body], 'touches fixture')):
            check = lf.Clearance(NS(radius=.4, noncutting_sections=()), (0, 0, 1),
                                 [] if fixtures else [body], fixtures, .02)
            check.measure = MagicMock()
            check.measure.measureMinimumDistance.return_value.value = .39
            result = check.point((0, 0, 0), .001)
            if expected:
                self.assertIn(expected, result)
            else:
                self.assertIsNone(result)

    def test_fixture_margin_does_not_spend_the_cutting_chord_tolerance(self):
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        body = NS(pointContainment=lambda p: outside)
        check = lf.Clearance(NS(radius=.4, noncutting_sections=()), (0, 0, 1), [], [body], .02)
        check.measure = MagicMock()
        check.measure.measureMinimumDistance.return_value.value = .4005
        self.assertIn('touches fixture', check.point((0, 0, 0), .001))
