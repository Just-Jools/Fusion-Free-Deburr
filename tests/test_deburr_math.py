import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'FusionFreeDeburr'))
import deburr_math as dm  # noqa: E402

Z = (0.0, 0.0, 1.0)


def dist(a, b):
    return math.sqrt(dm.dot(dm.sub(a, b), dm.sub(a, b)))


class BallCenterTests(unittest.TestCase):
    def test_zero_break_touches_edge(self):
        center, _ = dm.ball_center((0, 0, 0), (0, 0, 1), (1, 0, 0), 0.3, 0.0)
        self.assertAlmostEqual(dist(center, (0, 0, 0)), 0.3)

    def test_square_edge_break_width(self):
        # Top face z=0 (material x<0), side face x=0. Ball must cross the top
        # plane exactly `w` back from the edge.
        r, w = 0.3, 0.05
        center, _ = dm.ball_center((0, 0, 0), (0, 0, 1), (1, 0, 0), r, w)
        x_cut = center[0] - math.sqrt(r * r - center[2] ** 2)
        self.assertAlmostEqual(-x_cut, w)

    def test_contact_points_lie_on_ball_for_various_angles(self):
        r, w = 0.3, 0.04
        for deg in (30, 60, 90, 120, 150):
            a = math.radians(deg)
            n1 = (0, 0, 1)
            n2 = (math.sin(a), 0, math.cos(a))  # deg between normals
            center, u = dm.ball_center((1, 2, 3), n1, n2, r, w)
            for q in dm.contact_points((1, 2, 3), n1, n2, u, w):
                self.assertAlmostEqual(dist(q, center), r, msg=f'{deg} deg')
                self.assertAlmostEqual(dist(q, (1, 2, 3)), w, msg=f'{deg} deg')

    def test_tip_is_center_minus_radius(self):
        r, w = 0.3, 0.05
        center, _ = dm.ball_center((0, 0, 0), (0, 0, 1), (1, 0, 0), r, w)
        tip = dm.tool_tip((0, 0, 0), (0, 0, 1), (1, 0, 0), Z, r, w)
        self.assertAlmostEqual(tip[2], center[2] - r)

    def test_tangent_faces_rejected(self):
        with self.assertRaises(dm.DeburrError):
            dm.ball_center((0, 0, 0), (0, 0, 1), (0.01, 0, 1), 0.3, 0.05)

    def test_break_too_large_rejected(self):
        with self.assertRaises(dm.DeburrError):
            dm.ball_center((0, 0, 0), (0, 0, 1), (1, 0, 0), 0.1, 0.5)

    def test_undercut_rejected(self):
        # Bottom edge of an overhang: bottom face n=-z, side n=+x.
        with self.assertRaises(dm.DeburrError):
            dm.tool_tip((0, 0, 0), (0, 0, -1), (1, 0, 0), Z, 0.3, 0.05)

    def test_lollipop_reaches_bottom_edge(self):
        r, w = .4, .02
        limit = dm.lollipop_contact_height(r, .2, .8)
        p, n1, n2 = (0, 0, 0), (0, 0, -1), (1, 0, 0)
        tip = dm.tool_tip(p, n1, n2, Z, r, w, limit)
        center, u = dm.ball_center(p, n1, n2, r, w)
        self.assertAlmostEqual(dist(dm.add(tip, dm.scale(Z, r)), center), 0)
        for q in dm.contact_points(p, n1, n2, u, w):
            self.assertAlmostEqual(dist(q, center), r)

    def test_lollipop_neck_and_flutes_limit_contact(self):
        for neck, flutes in ((.39, .8), (.2, .45)):
            limit = dm.lollipop_contact_height(.4, neck, flutes)
            with self.assertRaises(dm.DeburrError):
                dm.tool_tip((0, 0, 0), (0, 0, -1), (1, 0, 0), Z, .4, .02, limit)

    def test_lollipop_invalid_dimensions(self):
        for values in ((0, .2, .8), (.4, 0, .8), (.4, .4, .8),
                       (.4, .2, .4), (.4, .2, float('nan'))):
            with self.assertRaises(dm.DeburrError):
                dm.lollipop_contact_height(*values)

    def test_lollipop_contact_rotates_with_setup(self):
        limit = dm.lollipop_contact_height(.4, .2, .8)
        a = dm.tool_tip((0, 0, 0), (0, 0, -1), (1, 0, 0), Z, .4, .02, limit)
        b = dm.tool_tip((0, 0, 0), (-1, 0, 0), (0, 0, 1), (1, 0, 0), .4, .02, limit)
        self.assertEqual(b, (a[2], a[1], a[0]))


if __name__ == '__main__':
    unittest.main()
