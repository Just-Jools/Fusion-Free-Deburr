"""Pure geometry for ball-endmill edge breaks. No Fusion imports, so it can be
tested outside Fusion. Vectors are (x, y, z) tuples; units are whatever the
caller uses consistently (Fusion internal units are cm).

Cross-section through a sharp convex edge at point P, with outward face
normals n1, n2 and half-angle alpha between them (cos(alpha) = |n1 + n2| / 2):
the ball center sits on the outward bisector u at distance s from P. The ball
then cuts each face back a distance w from the edge, where

    w = sqrt(R^2 - s^2 cos^2(alpha)) - s sin(alpha)

Solving for s:

    s = -w sin(alpha) + sqrt(R^2 - w^2 cos^2(alpha))
"""

import math

MIN_EDGE_ANGLE_DEG = 10.0
_EPS = 1e-9


class DeburrError(Exception):
    pass


def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def scale(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def normalize(a):
    length = math.sqrt(dot(a, a))
    if length < _EPS:
        raise DeburrError('zero-length vector')
    return scale(a, 1.0 / length)


def center_distance(radius, width, cos_a, sin_a):
    """Distance s from the edge to the ball center along the bisector."""
    disc = radius * radius - (width * cos_a) ** 2
    if disc < 0:
        raise DeburrError('edge break is too large for this ball')
    s = -width * sin_a + math.sqrt(disc)
    if s <= 0:
        raise DeburrError('edge break is too large for this ball')
    return s


def ball_center(p, n1, n2, radius, width):
    """Ball center that cuts an edge break of `width` at edge point p.

    Returns (center, u) where u is the unit outward bisector.
    """
    n1, n2 = normalize(n1), normalize(n2)
    c = max(-1.0, min(1.0, dot(n1, n2)))
    if math.degrees(math.acos(c)) < MIN_EDGE_ANGLE_DEG:
        raise DeburrError('faces are nearly tangent (not a sharp edge)')

    u = normalize(add(n1, n2))
    cos_a = math.sqrt((1 + c) / 2)
    sin_a = math.sqrt((1 - c) / 2)
    s = center_distance(radius, width, cos_a, sin_a)
    return add(p, scale(u, s)), u


def contact_points(p, n1, n2, u, width):
    """Where the cut meets each face: `width` back from the edge, in-face."""
    points = []
    for n in (normalize(n1), normalize(n2)):
        m = normalize(sub(u, scale(n, dot(u, n))))  # in-face, pointing off the edge
        points.append(sub(p, scale(m, width)))
    return points


def lollipop_contact_height(radius, neck_radius, flute_length):
    """Highest usable spherical contact above the center, in length units.

    The neck intersects the sphere at sqrt(R² - neck²). Flute length is
    measured from the tool tip, so it can further restrict that height.
    This is a cutting-envelope limit, not a shaft/holder clearance test.
    """
    values = (radius, neck_radius, flute_length)
    if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
            or radius <= 0 or not 0 < neck_radius < radius or flute_length <= radius):
        raise DeburrError('lollipop requires a smaller positive neck and flutes above the ball center')
    return min(math.sqrt(radius * radius - neck_radius * neck_radius),
               flute_length - radius)


def tool_tip(p, n1, n2, tool_axis, radius, width, max_contact_height=0.0):
    """Tool tip position (center - R * axis) for a vertical-axis ball endmill.

    By default only the lower hemisphere cuts. A lollipop may supply its
    validated upper contact height. Clearance must be checked separately.
    """
    z = normalize(tool_axis)
    center, u = ball_center(p, n1, n2, radius, width)
    for q in contact_points(p, n1, n2, u, width):
        if dot(sub(q, center), z) > max_contact_height + 1e-6:
            raise DeburrError('edge is not reachable along the tool axis (undercut or neck contact)')
    return sub(center, scale(z, radius))
