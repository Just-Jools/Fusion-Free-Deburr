import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'FusionFreeDeburr'))
import deburr_chain as dc  # noqa: E402
import deburr_math as dm  # noqa: E402

Z = (0.0, 0.0, 1.0)
R, W = 0.3, 0.05
TOP = 'top'


def dist(a, b):
    return math.sqrt(dm.dot(dm.sub(a, b), dm.sub(a, b)))


def straight_segment(p0, p1, wall_normal, wall_id, n=1):
    """Top edge from p0 to p1 between the top face (+Z) and a wall."""
    pts = [dm.add(p0, dm.scale(dm.sub(p1, p0), k / n)) for k in range(n + 1)]
    tips = [dm.tool_tip(p, Z, wall_normal, Z, R, W) for p in pts]
    normals = {TOP: Z, wall_id: wall_normal}
    return dc.Segment(tips=tips, start=dc.End(p0, normals), end=dc.End(p1, normals),
                      start_key=p0, end_key=p1)


def square(walls_outward):
    """Four top edges of a 10x10 square. walls_outward=True: block (convex
    corners). False: pocket (walls face inward, concave corners for the path)."""
    c = [(0, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0)]
    sign = 1 if walls_outward else -1
    normals = [(0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)]
    return [straight_segment(c[k], c[(k + 1) % 4], dm.scale(normals[k], sign), f'wall{k}') for k in range(4)]


def assert_connected(test, chain):
    pieces = chain.pieces
    for a, b in zip(pieces, pieces[1:]):
        test.assertLess(dist(a.points[-1], b.points[0]), 1e-9)
    if chain.closed:
        test.assertLess(dist(pieces[-1].points[-1], pieces[0].points[0]), 1e-9)


class OrderChainsTests(unittest.TestCase):
    def test_shuffled_reversed_square_forms_one_closed_chain(self):
        segs = square(True)
        shuffled = [segs[2], segs[0].reversed(), segs[3], segs[1].reversed()]
        chains = dc.order_chains(shuffled)
        self.assertEqual(len(chains), 1)
        chain, closed = chains[0]
        self.assertTrue(closed)
        self.assertEqual(len(chain), 4)
        for a, b in zip(chain, chain[1:]):
            self.assertEqual(a.end_key, b.start_key)

    def test_open_chain_starts_at_dangling_end(self):
        segs = square(True)[:3]
        chains = dc.order_chains([segs[1], segs[2], segs[0]])
        self.assertEqual(len(chains), 1)
        chain, closed = chains[0]
        self.assertFalse(closed)
        self.assertEqual(len(chain), 3)

    def test_disconnected_edges_make_separate_chains(self):
        segs = square(True)
        chains = dc.order_chains([segs[0], segs[2]])
        self.assertEqual(len(chains), 2)


class LinkChainTests(unittest.TestCase):
    def test_block_corners_roll_around_vertex(self):
        chain, closed = dc.order_chains(square(True))[0]
        linked = dc.link_chain(chain, closed, Z, R, W)
        assert_connected(self, linked)
        self.assertEqual(len(linked.pieces), 8)  # 4 edges + 4 corner rolls
        # Corner roll keeps a constant distance from the corner (it's an arc).
        roll = linked.pieces[1].points
        corner = (10, 0, 0)
        radii = [math.hypot(p[0] - corner[0], p[1] - corner[1]) for p in roll]
        self.assertAlmostEqual(max(radii), min(radii), places=9)

    def test_pocket_corners_are_trimmed_to_crossing(self):
        chain, closed = dc.order_chains(square(False))[0]
        linked = dc.link_chain(chain, closed, Z, R, W)
        assert_connected(self, linked)
        self.assertEqual(len(linked.pieces), 4)
        d = abs(linked.pieces[0].points[0][1])  # inward offset of the path
        corner = linked.pieces[0].points[-1]
        self.assertAlmostEqual(corner[0], 10 - d)
        self.assertAlmostEqual(corner[1], d)

    def test_tangent_joint_is_merged(self):
        a = straight_segment((0, 0, 0), (5, 0, 0), (0, -1, 0), 'wall')
        b = straight_segment((5, 0, 0), (10, 0, 0), (0, -1, 0), 'wall')
        linked = dc.link_chain([a, b], False, Z, R, W)
        self.assertEqual(len(linked.pieces), 2)
        assert_connected(self, linked)

    def test_open_chain_has_no_wraparound_link(self):
        chain, closed = dc.order_chains(square(True)[:2])[0]
        linked = dc.link_chain(chain, closed, Z, R, W)
        self.assertFalse(linked.closed)
        self.assertEqual(len(linked.pieces), 3)  # 2 edges + 1 roll


class SplitBoundaryTests(unittest.TestCase):
    def test_edges_between_same_faces_are_bridged_without_warning(self):
        a = straight_segment((0, 0, 0), (5, 0, 0), (0, -1, 0), 'wall')
        b = straight_segment((5, 0, 0), (10, 0, 0), (0, -1, 0), 'wall')
        b.tips[0] = dm.add(b.tips[0], (0.001, 0, 0))  # small gap from normal evaluation
        linked = dc.link_chain([a, b], False, Z, R, W)
        self.assertEqual(linked.notes, [])
        assert_connected(self, linked)


class SpikeCornerTests(unittest.TestCase):
    """A needle of material pointing +Y: edge a runs up to the tip along +Y with
    a wall facing -X, edge b runs back down along -Y with a 45 deg chamfer facing
    +X. Both edges leave the tip in the same direction (-Y), so the shortest
    turn between the side normals would go over the top face."""

    def segments(self):
        s = math.sqrt(0.5)
        tip = (0.0, 0.0, 0.0)
        a_normals = {TOP: Z, 'wall': (-1.0, 0.0, 0.0)}
        b_normals = {TOP: Z, 'chamfer': (s, 0.0, s)}
        a_pts, b_pts = [(0.0, -5.0, 0.0), tip], [tip, (0.0, -5.0, 0.0)]
        a = dc.Segment(tips=[dm.tool_tip(p, Z, a_normals['wall'], Z, R, W) for p in a_pts],
                       start=dc.End(a_pts[0], a_normals, (0.0, 1.0, 0.0)),
                       end=dc.End(tip, a_normals, (0.0, -1.0, 0.0)), start_key='a0', end_key='tip')
        b = dc.Segment(tips=[dm.tool_tip(p, Z, b_normals['chamfer'], Z, R, W) for p in b_pts],
                       start=dc.End(tip, b_normals, (0.0, -1.0, 0.0)),
                       end=dc.End(b_pts[1], b_normals, (0.0, 1.0, 0.0)), start_key='tip', end_key='b1')
        return a, b

    def test_roll_goes_around_the_tip_not_over_the_top(self):
        a, b = self.segments()
        linked = dc.link_chain([a, b], False, Z, R, W)
        self.assertEqual(linked.notes, [])
        assert_connected(self, linked)
        roll = linked.pieces[1].points
        self.assertGreater(len(roll), 30)  # ~180 deg in 5 deg steps
        # Halfway round, the ball is past the tip (+Y), clear of the needle.
        self.assertGreater(roll[len(roll) // 2][1], 0.0)
        # Ball stays on the top face: center never lower than on the straight edges.
        lowest_edge = min(a.tips[-1][2], b.tips[0][2])
        for p in roll:
            self.assertGreaterEqual(p[2], lowest_edge - 1e-9)


class CornerChoiceTests(unittest.TestCase):
    def test_vertex_crossing_keeps_distance_from_vertex(self):
        a, b = SpikeCornerTests().segments()
        tip = (0.0, 0.0, 0.0)
        pts = dc.vertex_crossing(a.tips[-1], b.tips[0], tip, Z, R)
        self.assertEqual(pts[0], a.tips[-1])
        self.assertLess(dist(pts[-1], b.tips[0]), 1e-12)
        centers = [dm.add(p, dm.scale(Z, R)) for p in pts]
        d0, d1 = dist(centers[0], tip), dist(centers[-1], tip)
        for k, c in enumerate(centers):
            self.assertAlmostEqual(dist(c, tip), d0 + (d1 - d0) * k / (len(centers) - 1), places=9)
        # Goes over the vertex, not out past it.
        self.assertTrue(all(abs(p[1]) < 1e-9 for p in pts))

    def test_depth_function_picks_the_shallower_candidate(self):
        a, b = SpikeCornerTests().segments()
        # Material continues past the tip (+Y): anything out there cuts deep.
        depth = lambda tips: max(1.0 if p[1] > 1e-6 else 0.0 for p in tips)
        linked = dc.link_chain([a, b], False, Z, R, W, depth)
        self.assertEqual(linked.notes, [])
        self.assertTrue(all(p[1] <= 1e-6 for p in linked.pieces[1].points))
        # Nothing beyond the tip: the roll around the outside is kept.
        linked = dc.link_chain([a, b], False, Z, R, W, lambda tips: 0.0)
        self.assertGreater(linked.pieces[1].points[len(linked.pieces[1].points) // 2][1], 0.0)

    def test_note_when_best_corner_still_cuts_deep(self):
        a, b = SpikeCornerTests().segments()
        ends = (a.tips[-1], b.tips[0])
        depth = lambda tips: 0.0 if all(p in ends for p in tips) else 1.0
        linked = dc.link_chain([a, b], False, Z, R, W, depth)
        self.assertEqual(len(linked.notes), 1)
        self.assertIn('corner 1', linked.notes[0])


class PolylineTests(unittest.TestCase):
    def test_closed_chain_polyline_ends_on_start_without_duplicates(self):
        segs = square(walls_outward=True)
        linked = dc.link_chain(segs, True, Z, R, W)
        pts = dc.chain_points(linked)
        self.assertEqual(pts[-1], pts[0])
        for a, b in zip(pts, pts[1:]):
            self.assertGreater(dist(a, b), dc.JOIN_TOL)

    def test_open_chain_polyline_keeps_ends(self):
        segs = square(walls_outward=True)[:2]
        linked = dc.link_chain(segs, False, Z, R, W)
        pts = dc.chain_points(linked)
        self.assertEqual(pts[0], linked.pieces[0].points[0])
        self.assertEqual(pts[-1], linked.pieces[-1].points[-1])

    def test_knots_are_clamped_arc_length(self):
        pts = [(0, 0, 0), (3, 0, 0), (3, 4, 0)]
        self.assertEqual(dc.polyline_knots(pts), [0.0, 0.0, 3.0, 7.0, 7.0])
        self.assertEqual(len(dc.polyline_knots(pts)), len(pts) + 2)  # degree 1


if __name__ == '__main__':
    unittest.main()
