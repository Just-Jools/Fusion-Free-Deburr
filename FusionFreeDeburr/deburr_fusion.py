"""Fusion-side glue: sample B-rep edges, link them into continuous tool-tip
chains, draw those in a sketch, and create a Trace operation driven by them.
All lengths are in cm."""

import json
import math

import adsk.core
import adsk.fusion
import adsk.cam

from . import deburr_chain as dc
from . import deburr_math as dm

SAMPLE_STEP = 0.05          # max spacing between samples on curved edges (0.5 mm)
CONVEXITY_PROBE = 0.005     # step along the outward bisector to test convexity
VERTEX_TOL = 1e-4           # cm; vertices this close are treated as the same
TRACE_STRATEGY = 'trace'

ATTR_GROUP = 'AdditionalToolpaths'
ATTR_NAME = 'ballDeburr'


def _tup(v):
    return (v.x, v.y, v.z)


def _is_line_between_planes(edge: adsk.fusion.BRepEdge) -> bool:
    if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
        return False
    plane = adsk.core.SurfaceTypes.PlaneSurfaceType
    return all(f.geometry.surfaceType == plane for f in edge.faces)


def _is_closed(edge: adsk.fusion.BRepEdge) -> bool:
    return edge.startVertex is None or edge.startVertex == edge.endVertex


def _vertex_key(vertex: adsk.fusion.BRepVertex):
    return tuple(round(c / VERTEX_TOL) for c in _tup(vertex.geometry))


def _sample_edge(edge: adsk.fusion.BRepEdge, straight: bool):
    ev = edge.evaluator
    _, t0, t1 = ev.getParameterExtents()
    _, length = ev.getLengthAtParameter(t0, t1)
    n = 1 if straight else max(2, math.ceil(length / SAMPLE_STEP))
    points = []
    for i in range(n + 1):
        _, t = ev.getParameterAtLength(t0, length * i / n)
        _, p = ev.getPointAtParameter(t)
        points.append(_tup(p))
    return points


def _normal(face: adsk.fusion.BRepFace, p):
    ok, n = face.evaluator.getNormalAtPoint(adsk.core.Point3D.create(*p))
    if not ok:
        raise dm.DeburrError('could not evaluate face normal')
    return dm.normalize(_tup(n))


class _FaceIds:
    """Small integer ids for faces, so neighbouring edges can find their shared face."""

    def __init__(self):
        self._faces = []

    def __call__(self, face):
        for i, f in enumerate(self._faces):
            if f == face:
                return i
        self._faces.append(face)
        return len(self._faces) - 1


def edge_segment(edge: adsk.fusion.BRepEdge, radius, width, tool_axis, face_ids) -> dc.Segment:
    """Tool-tip path for one edge, oriented start vertex -> end vertex.
    Raises DeburrError if the edge can't be done."""
    if edge.faces.count != 2:
        raise dm.DeburrError('edge does not have exactly two faces')
    f1, f2 = edge.faces.item(0), edge.faces.item(1)
    id1, id2 = face_ids(f1), face_ids(f2)

    straight = _is_line_between_planes(edge)
    closed = _is_closed(edge)
    samples = _sample_edge(edge, straight)
    if closed:
        samples = samples[:-1]
    else:
        start = _tup(edge.startVertex.geometry)
        if dc.dist(samples[0], start) > dc.dist(samples[-1], start):
            samples.reverse()

    # Convexity check at the middle sample: just outside the edge along the
    # outward bisector must be outside the body.
    mid = samples[len(samples) // 2]
    u = dm.normalize(dm.add(_normal(f1, mid), _normal(f2, mid)))
    probe = dm.add(mid, dm.scale(u, CONVEXITY_PROBE))
    containment = edge.body.pointContainment(adsk.core.Point3D.create(*probe))
    if containment == adsk.fusion.PointContainment.PointInsidePointContainment:
        raise dm.DeburrError('edge is concave')

    # Edge direction at each end, pointing into the edge (for corner rolls).
    tangents = {0: dm.normalize(dm.sub(samples[1], samples[0])),
                len(samples) - 1: dm.normalize(dm.sub(samples[-2], samples[-1]))}
    tips, ends = [], []
    for k, p in enumerate(samples):
        n1, n2 = _normal(f1, p), _normal(f2, p)
        tips.append(dm.tool_tip(p, n1, n2, tool_axis, radius, width))
        if k in tangents:
            ends.append(dc.End(p, {id1: n1, id2: n2}, tangents[k]))

    return dc.Segment(
        tips=tips, start=ends[0], end=ends[-1],
        start_key=None if closed else _vertex_key(edge.startVertex),
        end_key=None if closed else _vertex_key(edge.endVertex),
        closed=closed, source=edge)


def _cut_depth(body: adsk.fusion.BRepBody, radius, tool_axis):
    """Function(tool tips) -> deepest cut of the ball into `body` along them
    (radius minus the ball center's distance to the body)."""
    measure = adsk.core.Application.get().measureManager
    z = dm.normalize(tool_axis)

    def depth(tips):
        worst = -radius
        for tip in tips:
            center = adsk.core.Point3D.create(*dm.add(tip, dm.scale(z, radius)))
            worst = max(worst, radius - measure.measureMinimumDistance(center, body).value)
        return worst
    return depth


def build_chains(edges, radius, width, tool_axis):
    """Returns (chains, used_edges, warnings). Connected edges are linked into
    continuous chains (see deburr_chain)."""
    face_ids = _FaceIds()
    segments, warnings = [], []
    for i, edge in enumerate(edges):
        try:
            segments.append(edge_segment(edge, radius, width, tool_axis, face_ids))
        except dm.DeburrError as e:
            warnings.append(f'Edge {i + 1} skipped: {e}')

    chains = []
    for ordered, closed in dc.order_chains(segments):
        depth = _cut_depth(ordered[0].source.body, radius, tool_axis)
        chain = dc.link_chain(ordered, closed, tool_axis, radius, width, depth)
        warnings += chain.notes
        if chain.pieces:
            chains.append(chain)
    return chains, [s.source for s in segments], warnings


def setup_tool_axis(setup: adsk.cam.Setup):
    _, _, _, z = setup.workCoordinateSystem.getAsCoordinateSystem()
    return _tup(z)


def build_sketch(design: adsk.fusion.Design, chains, name):
    """Draws each chain as a single curve: a degree-1 NURBS (exact polyline,
    corners stay sharp). Returns (sketch, [[curve] per chain]).

    One curve per chain, not one per piece: Trace's chain resolver mishandles
    multi-curve chains that are edge-on to the tool axis (e.g. a profile in a
    vertical plane) - it fails with "No contours have been specified" or, on
    regeneration, swaps the chain for another chain's curves."""
    root = design.rootComponent
    sketch = root.sketches.add(root.xYConstructionPlane)
    sketch.name = name
    sketch.isComputeDeferred = True
    splines = sketch.sketchCurves.sketchFixedSplines
    curve_chains = []
    try:
        for chain in chains:
            points = dc.chain_points(chain)
            ctrl = [sketch.modelToSketchSpace(adsk.core.Point3D.create(*p)) for p in points]
            nurbs = adsk.core.NurbsCurve3D.createNonRational(ctrl, 1, dc.polyline_knots(points), False)
            curve_chains.append([splines.addByNurbsCurve(nurbs)])
    finally:
        sketch.isComputeDeferred = False
    return sketch, curve_chains


def _curves_parameter(op_or_input):
    """The curve-selection parameter of a trace Operation or OperationInput."""
    params = op_or_input.parameters
    param = params.itemByName('curves')
    if param:
        value = adsk.cam.CadContours2dParameterValue.cast(param.value)
        if value:
            return value
    for param in params:
        value = adsk.cam.CadContours2dParameterValue.cast(param.value)
        if value:
            return value
    names = ', '.join(p.name for p in params)
    raise RuntimeError(f'No curve parameter on trace input. Parameters: {names}')


def _apply_curves(value: adsk.cam.CadContours2dParameterValue, curve_chains):
    """One chain selection per connected chain, so Trace runs it without retracts."""
    selections = value.getCurveSelections()
    selections.clear()
    for curves in curve_chains:
        chain = selections.createNewChainSelection()
        chain.inputGeometry = curves
    value.applyCurveSelections(selections)


def create_trace(setup: adsk.cam.Setup, curve_chains, name, tool=None):
    op_input = setup.operations.createInput(TRACE_STRATEGY)
    op_input.displayName = name
    if tool:
        op_input.tool = tool
    # Not generated here: the caller generates after its last change to the op.
    op_input.generationMode = adsk.cam.AutomaticGenerationModes.SkipGeneration
    op = setup.operations.add(op_input)
    # Curves go on the added operation (same path as editing), so create and
    # edit can't diverge in how the selection is stored.
    _apply_curves(_curves_parameter(op), curve_chains)
    return op


def update_trace_curves(op: adsk.cam.Operation, curve_chains):
    _apply_curves(_curves_parameter(op), curve_chains)


# --- Settings stored on the operation, so it can be edited later ---------

def save_settings(op: adsk.cam.Operation, edges, width, sketch):
    data = {
        'version': 1,
        'edges': [e.entityToken for e in edges],
        'width': width,
        'sketch': sketch.entityToken,
    }
    op.attributes.add(ATTR_GROUP, ATTR_NAME, json.dumps(data))


def load_settings(op: adsk.cam.Operation):
    attr = op.attributes.itemByName(ATTR_GROUP, ATTR_NAME)
    return json.loads(attr.value) if attr else None


def find_by_token(design: adsk.fusion.Design, token, cast):
    for entity in design.findEntityByToken(token) if token else []:
        typed = cast(entity)
        if typed:
            return typed
    return None


def operation_for_entity(design: adsk.fusion.Design, entity, ops):
    """The deburr op in `ops` that `entity` refers to: the op itself, or its
    sketch or one of its sketch curves (what a click on the path in the canvas
    picks, since the sketch lies on the toolpath). None if neither."""
    op = adsk.cam.Operation.cast(entity)
    if op:
        return next((o for o in ops if o == op), None)
    sketch = adsk.fusion.Sketch.cast(entity)
    if not sketch:
        curve = adsk.fusion.SketchCurve.cast(entity)
        sketch = curve.parentSketch if curve else None
    if not sketch:
        return None
    # A canvas pick in Manufacture is a proxy through the CAM model occurrence.
    while sketch.assemblyContext:
        sketch = sketch.nativeObject
    for o in ops:
        settings = load_settings(o)
        if settings and find_by_token(design, settings.get('sketch'), adsk.fusion.Sketch.cast) == sketch:
            return o
    return None


def milling_setups(cam: adsk.cam.CAM):
    return [s for s in cam.setups if s.operationType == adsk.cam.OperationTypes.MillingOperation] if cam else []


def deburr_operations(setups):
    """All operations in the given setups that were created by this add-in."""
    ops = []
    for setup in setups:
        for item in setup.allOperations:
            op = adsk.cam.Operation.cast(item)
            if op and op.attributes.itemByName(ATTR_GROUP, ATTR_NAME):
                ops.append(op)
    return ops
