# Geometry, safety, and validation notes

Ball endmills and spherical lollipop mills are supported, including when editing
an existing operation. Both require a positive finite diameter. Lollipop profiles also
require valid sphere radius, neck, flute, shoulder, and shaft dimensions. Trace compensation is centered, controller compensation is disabled
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
Trace contours. For ball endmills, corner ball-clearance checks sample straight spans at intervals no larger
than 0.5 mm or a quarter ball radius. They inspect the selected body only and do not prove
continuous clearance or check shank, holder, fixtures, other bodies, or Fusion's linking
moves. Simulate generated paths and inspect retracts before machining.

Lollipop contact is limited by both the neck's intersection with the spherical head
and the defined flute length. Corner rolls use the same contact limit; the generic
vertex-crossing fallback is disabled for lollipops because it does not model that limit.
Lollipop paths additionally check all selected solid models, fixtures, and other solid
design bodies. Noncutting neck, shaft, holder, and unfluted cap sections are bounded
by cylinders; tapered sections use the larger radius and may reject otherwise possible
cuts. Clearance is evaluated along linear spans at an initial spacing of at most 0.1 mm,
with inflated envelopes and finer subdivision near a clearance limit. Fixture checks
retain the clearance margin without allowing the cutting chord tolerance.

Each lollipop contour includes explicit approach and withdrawal outside the bounding
support plane of all obstacles. Its exterior endpoints differ even for a closed cutting
rim, so Trace cannot choose an interior seam. Leads, smoothing, depth ordering, and
Trace offsets are disabled; full retracts and selection order are preserved. These
checks use the finished solid model and the tool assembly as defined in the library.
They do not simulate remaining stock or unmodeled workholding. Repositioned manufacturing
models and non-solid setup selections are rejected.

Run regressions outside Fusion with `python -m unittest discover -s tests`.
Fusion API tests use mocks; actual Trace generation and parameter compatibility must
also be checked in Fusion.

The dialog's execute handler snapshots only the document, setup/operation, edge tokens,
tool JSON, and width. It performs no Design or CAM mutations. Once the dialog is
destroyed, a delayed custom event runs the update on Fusion's main thread when the
selection command is idle. Other commands defer the job until they terminate. A
changed document or invalidated setup, operation, or edge rejects the queued job.
Cancellation queues nothing; stopping the add-in cancels queued work. Cross-product
updates are not grouped into the dialog's single undo transaction. Explicit failure
rollback below still applies. This separation avoids nesting sketch/CAM edits in a
Manufacture command's commit; live verification is required after lifecycle changes.

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

Lollipop verification (2026-11-02, Fusion 2706.1.17): the four rim edges of the
showcase side pocket generated one Trace contour using the document's 8 mm
lollipop, 4 mm neck, and 20 mm shoulder length. The top-down setup was rejected
for shaft interference; the underside setup cleared with the same tool dimensions.
A queued update to 0.25 mm generated successfully and was saved as document version 6.
A subsequent normal dialog edit to 0.2 mm completed successfully with deferred execution
and was saved as document version 7.
All seven operations remained valid, the lollipop operation had no warnings or errors,
and sketch count remained 14. The exported 82 moves, including approach and retract,
passed clearance checks at 2,155 sampled positions. Generated feed vertices were
within 0.000067 mm of the stored sketch polyline (excluding the intentional final
feed retract). The diagnostic post emits CSV, not machine code. The tool definition
had no holder, so this check does not establish clearance for an added physical holder.
