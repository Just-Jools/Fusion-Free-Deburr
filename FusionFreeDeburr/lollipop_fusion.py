"""Lollipop paths with explicit side entry/exit and sampled tool clearance.

Trace approaches only the distant ends of these open curves. Each curve
includes its own feed approach and withdrawal outside the model bounds.
"""

import itertools
import math

import adsk.core
import adsk.fusion

from . import deburr_math as dm
from . import deburr_chain as dc
from . import deburr_fusion as df

CLEARANCE_STEP = .01  # cm, 0.1 mm maximum motion spacing
CLEARANCE_MARGIN = .001  # cm, 0.01 mm
CENTER_ENDPOINT_SEPARATION = .1  # cm; keep Trace from treating entry/exit as closed
SIDE_SURFACE_CLEARANCE = .05  # cm; ball envelope stays 0.5 mm off the body


def _same_body_placement(body, candidate):
    if (body.nativeObject or body) != (candidate.nativeObject or candidate):
        return False
    def transform(entity):
        context = entity.assemblyContext
        return (context.transform2.asArray() if context else
                [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    if any(abs(a - b) > 1e-6 for a, b in zip(transform(body), transform(candidate))):
        return False
    a, b = body.boundingBox, candidate.boundingBox
    return (dc.dist(df._tup(a.minPoint), df._tup(b.minPoint)) < 1e-6 and
            dc.dist(df._tup(a.maxPoint), df._tup(b.maxPoint)) < 1e-6)


def setup_edges(edges, bodies):
    """Resolve CAM picks into the design context used for clearance and tokens."""
    result = []
    for edge in edges:
        if edge.body in bodies:
            result.append(edge)
            continue
        body = next((b for b in bodies if _same_body_placement(edge.body, b)), None)
        if body is None:
            raise dm.DeburrError('selected lollipop edges must belong to the setup models')
        native = edge.nativeObject or edge
        mapped = (native.createForAssemblyContext(body.assemblyContext)
                  if body.assemblyContext else native)
        if mapped is None or mapped.body != body:
            raise dm.DeburrError('could not resolve selected lollipop edge in the setup model')
        result.append(mapped)
    return result


def setup_obstacles(setup, design):
    def expand(entities):
        result = []
        for entity in entities:
            body = adsk.fusion.BRepBody.cast(entity)
            occurrence = adsk.fusion.Occurrence.cast(entity)
            if body and body.isSolid:
                result.append(body)
            elif occurrence:
                result.extend(expand(occurrence.bRepBodies))
                result.extend(expand(occurrence.childOccurrences))
            else:
                raise dm.DeburrError('lollipop clearance requires solid setup models and fixtures')
        return result
    all_bodies = list(design.rootComponent.bRepBodies)
    for occurrence in design.rootComponent.allOccurrences:
        all_bodies.extend(occurrence.bRepBodies)
    def canonical(body):
        # CAM wraps even root bodies in a manufacturing occurrence. Resolve
        # that proxy by native identity AND placement, never just by name.
        for candidate in all_bodies:
            if _same_body_placement(body, candidate):
                return candidate
        raise dm.DeburrError('repositioned manufacturing models are not supported for lollipop paths')
    bodies = [canonical(b) for b in expand(setup.models)]
    fixtures = [canonical(b) for b in expand(setup.fixtures)]
    if not bodies:
        raise dm.DeburrError('select solid bodies in the milling setup')
    for body in all_bodies:
        if body.isSolid and body not in bodies and body not in fixtures:
            fixtures.append(body)
    return bodies, fixtures


def _box_corners(body):
    box = body.boundingBox
    return list(itertools.product(*zip(df._tup(box.minPoint), df._tup(box.maxPoint))))


def _overlap(a0, a1, b):
    box = b.boundingBox
    lo, hi = df._tup(box.minPoint), df._tup(box.maxPoint)
    return all(a0[k] <= hi[k] and a1[k] >= lo[k] for k in range(3))


def _cylinder_bounds(a, b, radius, axis):
    # A cylinder has no radial extent along its axis. Using radius in all
    # three coordinates needlessly intersects holders high above the part.
    extent = [radius * math.sqrt(max(0, 1 - axis[k] * axis[k])) for k in range(3)]
    return (tuple(min(a[k], b[k]) - extent[k] for k in range(3)),
            tuple(max(a[k], b[k]) + extent[k] for k in range(3)))


class Clearance:
    def __init__(self, cutter, axis, bodies, fixtures, width):
        self.cutter, self.axis = cutter, dm.normalize(axis)
        self.bodies, self.fixtures, self.width = list(bodies), list(fixtures), width
        self.measure = adsk.core.Application.get().measureManager
        self.temp = adsk.fusion.TemporaryBRepManager.get()
        self.cache = {}

    def point(self, tip, margin, cutting=True):
        key = (tip, margin, cutting)
        if key in self.cache:
            return self.cache[key]
        center = dm.add(tip, dm.scale(self.axis, self.cutter.radius))
        point = adsk.core.Point3D.create(*center)
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        obstacles = self.bodies + self.fixtures
        for body in obstacles:
            fixture = body in self.fixtures
            allowance = 0 if fixture or not cutting else self.width
            if body.pointContainment(point) != outside:
                self.cache[key] = 'ball center enters model or fixture'
                return self.cache[key]
            distance = self.measure.measureMinimumDistance(point, body).value
            tolerance = 0 if fixture else df.PATH_TOL
            if distance < self.cutter.radius + margin - allowance - tolerance:
                self.cache[key] = 'ball exceeds edge-break depth or touches fixture'
                return self.cache[key]
        for bottom, top, radius in self.cutter.noncutting_sections:
            # Cylinders bound tapered sections and their motion between samples.
            a = dm.add(tip, dm.scale(self.axis, bottom - margin))
            b = dm.add(tip, dm.scale(self.axis, top + margin))
            r = radius + margin
            lo, hi = _cylinder_bounds(a, b, r, self.axis)
            for body in obstacles:
                if not _overlap(lo, hi, body):
                    continue
                cylinder = self.temp.createCylinderOrCone(adsk.core.Point3D.create(*a), r,
                                                          adsk.core.Point3D.create(*b), r)
                if not cylinder or not self.temp.booleanOperation(
                        cylinder, body, adsk.fusion.BooleanTypes.IntersectionBooleanType):
                    raise dm.DeburrError('could not check lollipop shaft clearance')
                if cylinder.faces.count:
                    self.cache[key] = 'neck, shaft, or holder intersects model or fixture'
                    return self.cache[key]
        self.cache[key] = None
        return None

    def path(self, points, cutting=True):
        def span(a, b, depth=0):
            margin = dc.dist(a, b) / 2 + CLEARANCE_MARGIN
            for p in (a, b):
                error = self.point(p, margin, cutting)
                if not error:
                    continue
                # A close but valid cut can fail the coarse swept envelope.
                # Refine it rather than rejecting small edge-break widths.
                if self.point(p, CLEARANCE_MARGIN, cutting) or depth == 10:
                    return error
                midpoint = dm.scale(dm.add(a, b), .5)
                return span(a, midpoint, depth + 1) or span(midpoint, b, depth + 1)
            return None
        for a, b in zip(points, points[1:]):
            n = max(1, math.ceil(dc.dist(a, b) / CLEARANCE_STEP))
            for k in range(n):
                error = span(dm.add(a, dm.scale(dm.sub(b, a), k / n)),
                             dm.add(a, dm.scale(dm.sub(b, a), (k + 1) / n)))
                if error:
                    return error
        return None


def _directions(edges, axis):
    candidates = []
    for edge in edges:
        p = df._tup(edge.pointOnEdge)
        for face in edge.faces:
            n = df._normal(face, p)
            horizontal = dm.sub(n, dm.scale(axis, dm.dot(n, axis)))
            if dm.dot(horizontal, horizontal) > 1e-8:
                candidates.append(dm.normalize(horizontal))
    # Also allow access from above, for top faces and open pockets.
    candidates.append(axis)
    unique = []
    for n in candidates:
        if not any(dm.dot(n, other) > .999 for other in unique):
            unique.append(n)
    return unique


def _center_access(points, clearance):
    """Approach a closed internal rim radially from its clear centre.

    Every possible seam is checked because a bore may be partly obstructed.
    The withdrawal endpoint is displaced slightly along the tool axis so the
    complete approach/cut/withdrawal curve remains open for Trace.
    """
    if len(points) < 4 or dc.dist(points[0], points[-1]) > dc.JOIN_TOL:
        return None
    center = dc.closed_path_center(points)
    if clearance.point(center, CLEARANCE_MARGIN):
        return None

    exits = [dm.add(center, dm.scale(clearance.axis, sign * CENTER_ENDPOINT_SEPARATION))
             for sign in (1, -1)]
    ring = points[:-1]
    # Try the closest seam first; this normally gives the shortest radial move.
    order = sorted(range(len(ring)), key=lambda i: dc.dist(center, ring[i]))
    for i in order:
        cut = dc.rotate_closed_path(points, i)
        seam = cut[0]
        if clearance.path([center, seam]) is not None:
            continue
        for exit_point in exits:
            if (clearance.point(exit_point, CLEARANCE_MARGIN) is None and
                    clearance.path([seam, center, exit_point]) is None):
                return [center] + cut + [center, exit_point]
    return None


def _is_closed_side_loop(points, axis, radius):
    """True for a closed contour with meaningful travel along the tool axis."""
    if len(points) < 4 or dc.dist(points[0], points[-1]) > dc.JOIN_TOL:
        return False
    heights = [dm.dot(p, axis) for p in points[:-1]]
    return max(heights) - min(heights) > max(10 * df.PATH_TOL, .1 * radius)


def _loop_normal(points):
    """Area-weighted normal of a closed, approximately planar tool-tip loop."""
    normal = (0.0, 0.0, 0.0)
    for a, b in zip(points, points[1:] + points[:1]):
        normal = dm.add(normal, dm.cross(a, b))
    if dm.dot(normal, normal) < 1e-12:
        raise dm.DeburrError('could not determine the side-wall plane')
    return dm.normalize(normal)


def _side_helix(cut, outward, clear_offset):
    """One turn that follows the loop while feeding laterally into its plane."""
    spans = max(1, len(cut) - 1)
    return [dm.add(point, dm.scale(outward, clear_offset * (1 - k / spans)))
            for k, point in enumerate(cut)]


def _is_external_side_loop(points, contacts, directions, axis):
    """True when the gauge loop lies outside the selected planar wall face."""
    ring = points[:-1]
    contact_ring = contacts[:-1] if contacts else []
    if len(contact_ring) != len(ring):
        return False
    plane_normal = _loop_normal(ring)
    wall_normals = [dm.normalize(d) for d in directions
                    if abs(dm.dot(dm.normalize(d), axis)) < 1e-6 and
                    abs(dm.dot(dm.normalize(d), plane_normal)) > .5]
    if not wall_normals:
        return False
    offset = tuple(sum(point[k] - contact[k]
                       for point, contact in zip(ring, contact_ring)) / len(ring)
                   for k in range(3))
    return max(dm.dot(offset, normal) for normal in wall_normals) > df.PATH_TOL


def _curved_side_access(points, contacts, directions, clearance):
    """Enter a closed external side loop from off the wall with a smooth blend."""
    axis = clearance.axis
    ring = points[:-1]
    contact_ring = contacts[:-1] if contacts else []
    if len(contact_ring) != len(ring):
        raise dm.DeburrError('side-wall lead is missing its model-edge reference points')
    plane_normal = _loop_normal(ring)
    candidates = [dm.normalize(d) for d in directions
                  if abs(dm.dot(dm.normalize(d), axis)) < 1e-6]
    if not candidates:
        return None
    alignment = [abs(dm.dot(d, plane_normal)) for d in candidates]
    best = max(alignment)
    # Ignore radial normals from the other edge face.  The approach direction
    # must be normal to the plane of this side-wall loop.
    horizontal = [d for d, score in zip(candidates, alignment)
                  if score >= max(.9 * best, .5)]
    heights = [dm.dot(p, axis) for p in ring]
    # Enter at the top of the side-wall loop.  The explicit curve therefore
    # descends outside the wall, turns through the Y-Z blend, feeds laterally
    # into the top seam, and only then follows the X-Z cutting loop.
    seam_order = sorted(range(len(ring)), key=lambda i: heights[i], reverse=True)

    for i in seam_order:
        cut = dc.rotate_closed_path(points, i)
        seam = cut[0]
        contact = contact_ring[i]
        for outward in horizontal:
            # Horizontal position is referenced to the real selected edge:
            # ball centre = wall point + radius + 0.5 mm clearance.
            existing = dm.dot(dm.sub(seam, contact), outward)
            wall_clearance = clearance.cutter.radius + SIDE_SURFACE_CLEARANCE - existing
            # Give the helical turn a definite pitch measured from the final
            # gauge loop.  For a 6 mm ball this is 6.5 mm, even when the gauge
            # loop itself is already offset from the model wall.
            clear_offset = max(2 * clearance.cutter.radius + SIDE_SURFACE_CLEARANCE,
                               wall_clearance)
            helix = _side_helix(cut, outward, clear_offset)
            outer_top = helix[0]
            if clearance.point(outer_top, SIDE_SURFACE_CLEARANCE, cutting=False):
                continue
            descent = 2 * clearance.cutter.radius + SIDE_SURFACE_CLEARANCE
            entry = dm.add(outer_top, dm.scale(axis, descent))
            exit_ = dm.add(outer_top,
                           dm.scale(axis, descent + CENTER_ENDPOINT_SEPARATION))
            # The straight plunge is completely outside the wall.  The helix
            # is then an intentional progressive cutting entry into the loop.
            if clearance.path([entry, outer_top], cutting=False) is not None:
                continue
            path = [entry] + helix + cut[1:] + list(reversed(helix))[1:] + [exit_]
            if clearance.path(path) is None:
                return path
    return None


def add_access(points, directions, clearance, contacts=None):
    """Move horizontally (or upward) outside every obstacle before Trace links.

    Return an open curve even when the cutting contour is closed. Trace must
    start and finish at the exterior endpoints, rather than choose a seam.
    """
    side_loop = _is_closed_side_loop(points, clearance.axis, clearance.cutter.radius)
    # An external loop can also have a clear geometric centre, but using it
    # would select the old coplanar centre entry and bypass the 3D helix.
    if side_loop and _is_external_side_loop(
            points, contacts, directions, clearance.axis):
        side_path = _curved_side_access(points, contacts, directions, clearance)
        if side_path is not None:
            return side_path
        raise dm.DeburrError('no clear curved outside approach for this side-wall rim')

    center_path = _center_access(points, clearance)
    if center_path is not None:
        return center_path

    if side_loop:
        side_path = _curved_side_access(points, contacts, directions, clearance)
        if side_path is not None:
            return side_path
        raise dm.DeburrError('no clear curved outside approach for this side-wall rim')

    corners = [p for body in clearance.bodies + clearance.fixtures for p in _box_corners(body)]
    envelope = max([clearance.cutter.radius] +
                   [r for _, _, r in clearance.cutter.noncutting_sections])
    def access(point, extra=0):
        for direction in directions:
            reach = max(dm.dot(dm.sub(p, point), direction) for p in corners)
            distance = max(0, reach) + envelope + .2 + extra
            outer = dm.add(point, dm.scale(direction, distance))
            # Any shaft point above this endpoint stays outside the support
            # plane for a lateral direction or rises farther for an upward one.
            if clearance.path([outer, point]) is None:
                return outer
        raise dm.DeburrError('no clear lollipop approach/withdrawal; select an accessible rim')
    # Different exterior endpoints prevent Trace from treating the entire
    # approach/cut/withdrawal curve as a closed contour with a movable seam.
    return [access(points[0])] + points + [access(points[-1], .1)]


def build_chains(edges, cutter, width, tool_axis, bodies, fixtures=()):
    """Build and validate all selected edges; reject the operation on any failure."""
    if not edges or not math.isfinite(width) or width <= 0:
        raise dm.DeburrError('select edges and a positive finite edge break')
    axis = dm.normalize(tool_axis)
    edges = setup_edges(edges, bodies)
    clearance = Clearance(cutter, axis, bodies, fixtures, width)
    ids = df._FaceIds()
    segments = [df.edge_segment(e, cutter.radius, width, axis, ids, cutter) for e in edges]
    chains, notes = [], []
    directions = _directions(edges, axis)
    for ordered, closed in dc.order_chains(segments):
        depth = df._cut_depth(ordered[0].source.body, cutter.radius, axis)
        chain = dc.link_chain(ordered, closed, axis, cutter.radius, width, depth, cutter)
        notes.extend(chain.notes)
        paths = dc.chain_polylines(chain)
        contacts = (ordered[0].contacts if closed and len(ordered) == 1 and len(paths) == 1
                    else None)
        for points in paths:
            error = clearance.path(points)
            if error:
                raise dm.DeburrError('lollipop cutting path: ' + error)
            points = add_access(points, directions, clearance, contacts)
            chains.append(dc.Chain(pieces=[dc.Piece(points)], closed=False))
    return chains, edges, notes
