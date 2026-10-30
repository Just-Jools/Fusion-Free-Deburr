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
