# Fusion Free Deburr

Ball endmill deburr toolpaths for Autodesk Fusion. Early alpha: simulate before cutting.

## Install (Windows)

1. Open [the latest release](https://github.com/jkkicks/Fusion-Free-Deburr/releases) and download **Fusion-Free-Deburr-v0.1.0-alpha.zip** under **Assets**.
2. Extract the ZIP anywhere on your computer. Keep the extracted files there while using the add-in.
3. In Fusion, open **Utilities > Add-Ins > Scripts and Add-Ins** and click **+** next to **My Add-Ins**.
4. Select the extracted **FusionFreeDeburr** folder containing `FusionFreeDeburr.py` and `FusionFreeDeburr.manifest`, then run **FusionFreeDeburr**. Enable **Run on Startup** if desired.

If upgrading from AdditionalToolpaths, stop and remove its old entry in Scripts and Add-Ins before adding FusionFreeDeburr.

## Use

**Manufacture > Milling > 3D > Ball Deburr**: select the edges, pick a ball endmill, set the
edge break width. Right-click the operation to edit it.

## Build a release ZIP

Run `python tools/package.py`. The ZIP is written to `dist/` and contains the installable
`FusionFreeDeburr` folder, these instructions, and the license.

## License

[MIT](LICENSE)

## Geometry and safety limits

Only ball endmills with a positive finite diameter are supported, including when editing
an existing operation. Trace compensation is centered, controller compensation is disabled
when exposed, and axial/radial offsets are zeroed. Missing required Trace settings stop the
operation update rather than relying on user defaults.

Convexity uses lateral body-containment probes at sampled edge points; ambiguous or
obstructed edges are skipped. Curves use Fusion's adaptive strokes with a 0.01 mm chord
tolerance. This bounds edge geometry error, not the error of the offset tool-tip path on
rapidly changing surfaces. Very small features near the probe size (0.05 mm, retried at
0.005 mm for ambiguous containment) may be rejected.

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
