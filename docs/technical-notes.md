# Geometry, safety, and validation notes

Only ball endmills with a positive finite diameter are supported, including when editing
an existing operation. Trace compensation is centered, controller compensation is disabled
when exposed, and axial/radial offsets are zeroed. Missing required Trace settings stop the
operation update rather than relying on user defaults.

Convexity uses lateral body-containment probes at sampled edge points; ambiguous or
obstructed edges are skipped. Curves use Fusion's adaptive strokes with a 0.01 mm chord
tolerance. Non-planar edges also split spans longer than 0.5 mm or whose computed
tool-tip midpoint differs from the chord midpoint by more than 0.01 mm. Straight
edges between planes keep just their endpoints. Refinement stops after 12 levels;
edges that still miss tolerance, and closed edges with too few distinct samples,
are rejected. Midpoint checks are not a continuous error guarantee for surfaces
varying sharply between samples. Very small features near the probe size (0.05 mm,
retried at 0.005 mm for ambiguous containment) may be rejected.

Crossings must coincide in 3D within 0.001 mm. Failed corner construction or clearance
checks and connectors cutting more than the allowed excess depth are split into separate
Trace contours. Corner ball-clearance checks sample straight spans at intervals no larger
than 0.5 mm or a quarter ball radius. They inspect the selected body only and do not prove
continuous clearance or check shank, holder, fixtures, other bodies, or Fusion's linking
moves. Simulate generated paths and inspect retracts before machining.

Run regressions outside Fusion with `python -m unittest discover -s tests`.
Fusion API tests use mocks; actual Trace generation and parameter compatibility must
also be checked in Fusion.

Live verification (2026-10-30, Fusion 2705.1.30): the 11 saved edges in
`Xometry_Mill_Test_Part` produced one contour and generated a valid Trace toolpath
without errors or warnings. Editing reset a deliberately changed 1 mm axial offset
to zero. A flat endmill was rejected using an actual Fusion Tool object. This is
one model check, not exhaustive coverage of geometry or machine clearance.

Failed sketch builds remove partial geometry. Failed edits restore the previous
curve selection, tool, Trace offsets, saved settings, and sketch name before removing
the replacement sketch. The old sketch is deleted only after configuration succeeds.
Rollback does not restore Fusion's cached toolpath; regenerate after a failed edit.
If Fusion itself refuses restoration, both sketches are retained with an explicit
error so potentially referenced geometry is not deleted. Toolpath generation remains
asynchronous; later generation errors are reported by Fusion.

Additional live verification: an injected settings-save failure restored the old
sketch and settings without increasing sketch count; an injected spline-build failure
also left no partial sketch. A successful replacement kept the sketch count unchanged
and regenerated a valid toolpath without errors or warnings. A newly created Trace
also generated successfully; its temporary test operation was removed afterward.
