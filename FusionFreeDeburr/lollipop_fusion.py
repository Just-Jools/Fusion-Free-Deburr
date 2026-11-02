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
        native = body.nativeObject or body
        for candidate in all_bodies:
            if (candidate.nativeObject or candidate) != native:
                continue
            a, b = body.boundingBox, candidate.boundingBox
            if (dc.dist(df._tup(a.minPoint), df._tup(b.minPoint)) < 1e-6 and
                    dc.dist(df._tup(a.maxPoint), df._tup(b.maxPoint)) < 1e-6):
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


class Clearance:
    def __init__(self, cutter, axis, bodies, fixtures, width):
        self.cutter, self.axis = cutter, dm.normalize(axis)
        self.bodies, self.fixtures, self.width = list(bodies), list(fixtures), width
        self.measure = adsk.core.Application.get().measureManager
        self.temp = adsk.fusion.TemporaryBRepManager.get()
        self.cache = {}

    def point(self, tip, margin):
        key = (tip, margin)
        if key in self.cache:
            return self.cache[key]
        center = dm.add(tip, dm.scale(self.axis, self.cutter.radius))
        point = adsk.core.Point3D.create(*center)
        outside = adsk.fusion.PointContainment.PointOutsidePointContainment
        obstacles = self.bodies + self.fixtures
        for body in obstacles:
            fixture = body in self.fixtures
            allowance = 0 if fixture else self.width
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
            lo = tuple(min(a[k], b[k]) - r for k in range(3))
            hi = tuple(max(a[k], b[k]) + r for k in range(3))
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

    def path(self, points):
        def span(a, b, depth=0):
            margin = dc.dist(a, b) / 2 + CLEARANCE_MARGIN
            for p in (a, b):
                error = self.point(p, margin)
                if not error:
                    continue
                # A close but valid cut can fail the coarse swept envelope.
                # Refine it rather than rejecting small edge-break widths.
                if self.point(p, CLEARANCE_MARGIN) or depth == 10:
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


def add_access(points, directions, clearance):
    """Move horizontally (or upward) outside every obstacle before Trace links.

    Return an open curve even when the cutting contour is closed. Trace must
    start and finish at the exterior endpoints, rather than choose a seam.
    """
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
    clearance = Clearance(cutter, axis, bodies, fixtures, width)
    if any(edge.body not in clearance.bodies for edge in edges):
        raise dm.DeburrError('selected lollipop edges must belong to the setup models')
    ids = df._FaceIds()
    segments = [df.edge_segment(e, cutter.radius, width, axis, ids, cutter) for e in edges]
    chains, notes = [], []
    directions = _directions(edges, axis)
    for ordered, closed in dc.order_chains(segments):
        depth = df._cut_depth(ordered[0].source.body, cutter.radius, axis)
        chain = dc.link_chain(ordered, closed, axis, cutter.radius, width, depth, cutter)
        notes.extend(chain.notes)
        for points in dc.chain_polylines(chain):
            error = clearance.path(points)
            if error:
                raise dm.DeburrError('lollipop cutting path: ' + error)
            points = add_access(points, directions, clearance)
            chains.append(dc.Chain(pieces=[dc.Piece(points)], closed=False))
    return chains, edges, notes
