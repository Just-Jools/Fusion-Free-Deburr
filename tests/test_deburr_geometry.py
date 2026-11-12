import importlib
import math
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'FusionFreeDeburr'))

import deburr_chain as dc
import deburr_math as dm


class _AdskModule(types.ModuleType):
    def __getattr__(self, name):
        value = type(name, (), {})
        setattr(self, name, value)
        return value


adsk = types.ModuleType('adsk')
adsk.core = _AdskModule('adsk.core')
adsk.fusion = _AdskModule('adsk.fusion')
adsk.cam = _AdskModule('adsk.cam')
sys.modules.update({
    'adsk': adsk,
    'adsk.core': adsk.core,
    'adsk.fusion': adsk.fusion,
    'adsk.cam': adsk.cam,
})

package = types.ModuleType('fusion_free_deburr')
package.__path__ = [str(ROOT / 'FusionFreeDeburr')]
sys.modules['fusion_free_deburr'] = package
lf = importlib.import_module('fusion_free_deburr.lollipop_fusion')


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class DeburrGeometryTests(unittest.TestCase):
    def test_vertical_hole_uses_gauge_point_loop_not_model_edge(self):
        """A vertical circular rim produces a smaller loop shifted down in Z.

        The constant face normal is +Y.  The cylindrical face normal points
        from the solid into the hole, opposite the radial edge direction.
        """
        edge_radius = 2.0
        ball_radius = 0.6
        face = (0.0, 1.0, 0.0)
        axis = (0.0, 0.0, 1.0)
        projected_radius = edge_radius - ball_radius / math.sqrt(2.0)

        cases = (
            ((edge_radius, 0.0, 0.0), (-1.0, 0.0, 0.0),
             (projected_radius, ball_radius / math.sqrt(2.0), -ball_radius)),
            ((0.0, 0.0, edge_radius), (0.0, 0.0, -1.0),
             (0.0, ball_radius / math.sqrt(2.0),
              projected_radius - ball_radius)),
            ((-edge_radius, 0.0, 0.0), (1.0, 0.0, 0.0),
             (-projected_radius, ball_radius / math.sqrt(2.0), -ball_radius)),
            ((0.0, 0.0, -edge_radius), (0.0, 0.0, 1.0),
             (0.0, ball_radius / math.sqrt(2.0),
              -projected_radius - ball_radius)),
        )
        for edge, bore_normal, expected in cases:
            actual = dm.tool_tip(edge, face, bore_normal, axis, ball_radius, 0.0,
                                 max_contact_height=ball_radius)
            self.assertTrue(close(actual, expected), (actual, expected))


    def test_closed_path_center_and_seam_rotation(self):
        path = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0)]
        self.assertTrue(close(dc.closed_path_center(path), (0.0, 0.0, 0.0)))
        rotated = dc.rotate_closed_path(path, 2)
        self.assertEqual(rotated[0], (-1.0, 0.0, 0.0))
        self.assertEqual(rotated[-1], rotated[0])
        self.assertEqual(len(rotated), len(path))

    def test_closed_rim_enters_and_withdraws_through_its_center(self):
        path = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0)]

        class Clear:
            axis = (0.0, 0.0, 1.0)

            def point(self, point, margin, cutting=True):
                return None

            def path(self, points):
                return None

        result = lf._center_access(path, Clear())
        center = (0.0, 0.0, 0.0)
        self.assertEqual(result[0], center)
        self.assertEqual(result[-2], center)
        self.assertTrue(close(result[-1], (0.0, 0.0, lf.CENTER_ENDPOINT_SEPARATION)))
        self.assertEqual(result[1], result[-3])

    def test_open_path_does_not_get_center_access(self):
        class Clear:
            axis = (0.0, 0.0, 1.0)

        path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                (2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
        self.assertIsNone(lf._center_access(path, Clear()))

    def test_external_side_loop_descends_clear_then_helices_into_cut(self):
        path = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0)]
        contacts = list(path)

        class Cutter:
            radius = .3
            noncutting_sections = ()

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()
            bodies = ()
            fixtures = ()

            def point(self, point, margin, cutting=True):
                if cutting:
                    return 'centre is solid'
                return None if point[1] >= margin else 'ball is too close to body'

            def path(self, points, cutting=True):
                return None

        # The radial X candidate comes first deliberately: the loop-plane
        # normal must win even when another face normal is also clear.
        result = lf.add_access(path, [(1.0, 0.0, 0.0),
                                     (0.0, 1.0, 0.0), Clear.axis], Clear(), contacts)
        clear_offset = lf.DEFAULT_SIDE_CLEARANCE
        descent = 2 * Cutter.radius + clear_offset
        self.assertTrue(close(result[0],
                              (0.0, clear_offset, 1.0 + descent), 1e-6))
        self.assertAlmostEqual(result[0][0], result[1][0], places=9)
        self.assertAlmostEqual(result[0][1], result[1][1], places=9)
        self.assertGreater(result[0][2], result[1][2])
        self.assertAlmostEqual(result[-1][1], clear_offset, places=6)
        self.assertGreater(result[-1][2], result[0][2])
        self.assertGreater(len(result), len(path) + 2)
        seam = max(path[:-1], key=lambda p: p[2])
        self.assertIn(seam, result)

        # The default lead is one quarter-turn, not a complete revolution.
        helix = result[1:3]
        self.assertTrue(close(helix[0], (0.0, clear_offset, 1.0)))
        self.assertTrue(close(helix[-1], (-1.0, 0.0, 0.0)))
        self.assertTrue(all(a[1] >= b[1] for a, b in zip(helix, helix[1:])))
        # The following complete turn is flat in the X-Z cutting plane.
        flat_turn = result[2:7]
        self.assertEqual(flat_turn[0], flat_turn[-1])
        self.assertTrue(all(abs(p[1]) < 1e-9 for p in flat_turn))

    def test_side_lead_angle_controls_only_the_entry_arc(self):
        path = [(0.0, 0.0, 1.0), (-1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0), (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0)]
        lead, cut = lf._partial_loop(path, 1 / 8)
        self.assertEqual(lead[0], path[0])
        self.assertTrue(close(lead[-1], (-.5, 0.0, .5)))
        self.assertEqual(cut[0], lead[-1])
        self.assertEqual(cut[-1], lead[-1])
        self.assertIn(path[0], cut)

    def test_closed_top_loop_keeps_existing_straight_access(self):
        path = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0),
                (1.0, 0.0, 0.0)]

        class Box:
            class Bounds:
                minPoint = types.SimpleNamespace(x=-1.0, y=-1.0, z=-.1)
                maxPoint = types.SimpleNamespace(x=1.0, y=1.0, z=.1)
            boundingBox = Bounds()

        class Cutter:
            radius = .3
            noncutting_sections = ()

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()
            bodies = (Box(),)
            fixtures = ()

            def point(self, point, margin, cutting=True):
                return 'centre is solid'

            def path(self, points, cutting=True):
                return None

        result = lf.add_access(path, [(0.0, 1.0, 0.0), Clear.axis], Clear())
        self.assertEqual(len(result), len(path) + 2)
        self.assertEqual(result[1:-1], path)

    def test_blocked_curved_side_entry_fails_instead_of_crossing_part(self):
        path = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0)]
        contacts = list(path)

        class Cutter:
            radius = .3

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()

            def point(self, point, margin, cutting=True):
                return 'centre is solid'

            def path(self, points, cutting=True):
                return 'fixture collision'

        with self.assertRaisesRegex(lf.dm.DeburrError, 'no clear curved outside approach'):
            lf.add_access(path, [(0.0, 1.0, 0.0), Clear.axis], Clear(), contacts)

    def test_vertical_side_entry_joins_the_top_of_the_loop(self):
        path = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0)]
        contacts = list(path)

        class Cutter:
            radius = .3
            noncutting_sections = ()

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()
            bodies = ()
            fixtures = ()

            def point(self, point, margin, cutting=True):
                if cutting:
                    return 'centre is solid'
                return None if point[1] >= margin else 'ball is too close to body'

            def path(self, points, cutting=True):
                return None

        result = lf.add_access(path, [(0.0, 1.0, 0.0), Clear.axis], Clear(), contacts)
        self.assertAlmostEqual(result[1][2], 1.0)
        self.assertAlmostEqual(result[0][1],
                               lf.DEFAULT_SIDE_CLEARANCE,
                               places=6)

    def test_side_entry_clearance_is_referenced_to_the_compensated_sketch(self):
        path = [(1.0, .2, 0.0), (0.0, .2, 1.0),
                (-1.0, .2, 0.0), (0.0, .2, -1.0),
                (1.0, .2, 0.0)]
        contacts = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                    (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                    (1.0, 0.0, 0.0)]

        class Cutter:
            radius = .3
            noncutting_sections = ()

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()
            bodies = ()
            fixtures = ()

            def point(self, point, margin, cutting=True):
                if cutting:
                    return 'centre is solid'
                return None

            def path(self, points, cutting=True):
                return None

        requested = .08
        result = lf.add_access(path, [(0.0, 1.0, 0.0), Clear.axis], Clear(),
                               contacts, requested, math.pi / 2)
        # The sketch is already at Y=.2; the requested 0.8 mm is added once.
        self.assertAlmostEqual(result[1][1], .2 + requested, places=9)
        self.assertAlmostEqual(result[-2][1], .2 + requested, places=9)

    def test_external_side_loop_does_not_get_stolen_by_clear_center(self):
        path = [(1.0, .2, 0.0), (0.0, .2, 1.0),
                (-1.0, .2, 0.0), (0.0, .2, -1.0),
                (1.0, .2, 0.0)]
        contacts = [(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                    (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0),
                    (1.0, 0.0, 0.0)]

        class Cutter:
            radius = .3
            noncutting_sections = ()

        class Clear:
            axis = (0.0, 0.0, 1.0)
            cutter = Cutter()
            bodies = ()
            fixtures = ()

            def point(self, point, margin, cutting=True):
                return None

            def path(self, points, cutting=True):
                return None

        result = lf.add_access(path, [(0.0, 1.0, 0.0), Clear.axis],
                               Clear(), contacts)
        self.assertGreater(result[1][1], path[0][1])
        self.assertNotEqual(result[0], dc.closed_path_center(path))


if __name__ == '__main__':
    unittest.main()
