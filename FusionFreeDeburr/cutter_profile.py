"""Spherical cutting and noncutting envelopes from Fusion tool JSON, in cm.

The Fusion adapter validates shaft/holder interference and entry/retract
motion before using these profiles for CAM generation.
"""

from dataclasses import dataclass
import math

from . import deburr_math as dm


@dataclass(frozen=True)
class SphericalCutter:
    radius: float
    max_contact_height: float
    # Conservative cylinders (bottom height, top height, radius) from tip.
    noncutting_sections: tuple = ()

    def tip(self, p, n1, n2, tool_axis, width):
        return dm.tool_tip(p, n1, n2, tool_axis, self.radius, width,
                           self.max_contact_height)


def from_tool_json(raw):
    """Read only ball and lollipop mills; never infer a tool from its name."""
    scale = {'millimeters': .1, 'inches': 2.54}.get(raw.get('unit'))
    if scale is None:
        raise dm.DeburrError('unsupported tool units')
    g = raw.get('geometry', {})

    def length(key):
        v = g.get(key)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise dm.DeburrError('missing or invalid tool dimension: ' + key)
        return v * scale

    kind = raw.get('type')
    if kind not in ('ball end mill', 'lollipop mill'):
        raise dm.DeburrError('select a ball endmill or lollipop mill')
    radius = length('DC') / 2
    if kind == 'ball end mill':
        return SphericalCutter(radius, 0.0)
    if not math.isclose(length('RE'), radius, rel_tol=1e-6, abs_tol=1e-8):
        raise dm.DeburrError('lollipop must have a spherical head')
    neck = length('shoulder-diameter') / 2
    limit = dm.lollipop_contact_height(radius, neck,
                                      length('LCF'))
    shoulder, overall = length('shoulder-length'), length('OAL')
    neck_start = radius + math.sqrt(radius * radius - neck * neck)
    if not neck_start < shoulder < overall:
        raise dm.DeburrError('invalid lollipop shoulder or overall length')
    sections = [(neck_start, shoulder, neck)]
    if radius + limit < neck_start:
        sections.append((radius + limit, neck_start,
                         math.sqrt(radius * radius - limit * limit)))
    sections.append((shoulder, overall, length('SFDM') / 2))

    def add_segments(profile, start, section_scale=scale):
        for segment in profile.get('segments', []):
            vals = [segment.get(k) for k in ('height', 'lower-diameter', 'upper-diameter')]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in vals):
                raise dm.DeburrError('invalid shaft or holder segment')
            height, lower, upper = [v * section_scale for v in vals]
            sections.append((start, start + height, max(lower, upper) / 2))
            start += height

    add_segments(raw.get('shaft', {}), shoulder)
    holder = raw.get('holder', {})
    if holder.get('segments'):
        holder_scale = {'millimeters': .1, 'inches': 2.54}.get(holder.get('unit'))
        if holder_scale is None:
            raise dm.DeburrError('unsupported holder units')
        gauge = holder.get('gaugeLength')
        assembly = g.get('assemblyGaugeLength')
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in (gauge, assembly)):
            raise dm.DeburrError('missing holder assembly gauge length')
        stickout = assembly * scale - gauge * holder_scale
        if stickout <= shoulder:
            raise dm.DeburrError('holder begins below the tool shoulder')
        add_segments(holder, stickout, holder_scale)
    return SphericalCutter(radius, limit, tuple(sections))
