<div align="center">

# Fusion Free Deburr

[![Version](https://img.shields.io/badge/version-0.1.0--alpha-blue)](https://github.com/jkkicks/Fusion-Free-Deburr/releases) [![License](https://img.shields.io/badge/license-MIT-green)](LICENSE) ![Status: alpha](https://img.shields.io/badge/status-alpha-orange)

**Free 3-axis deburring toolpaths for Autodesk Fusion, using a ball end mill.**

[Download the latest release](https://github.com/jkkicks/Fusion-Free-Deburr/releases)

</div>

![Fusion simulation of ball deburring on the showcase part](docs/images/readme/simulation.gif)

*Fusion simulation of the showcase part.*

![Selected edges and resulting toolpaths](docs/images/readme/workflow-select-edges-to-toolpath.gif)

*Selected edges and their resulting toolpaths.*

## Examples

| Curved edges and varying Z | Freeform pocket |
| :---: | :---: |
| ![Perimeter toolpath](docs/images/readme/gallery-varying-z-perimeter-side.png) | ![Pocket toolpath](docs/images/readme/gallery-freeform-pocket.png) |
| **Saddle holes** | **Holes on a slope** |
| ![Ridge hole toolpaths](docs/images/readme/gallery-ridge-saddle-holes.png) | ![Slope hole toolpaths](docs/images/readme/gallery-holes-normal-to-25deg-slope.png) |

## Install

1. Download the ZIP from the [latest release](https://github.com/jkkicks/Fusion-Free-Deburr/releases) and extract it.
2. In Fusion, press **Shift+S** to open **Scripts and Add-Ins**, or find it under **Utilities > Add-Ins**.
3. Click **+** beside **My Add-Ins**, select the extracted `FusionFreeDeburr` folder, and run the add-in.

## Use

In **Manufacture > Milling > 3D > Ball Deburr**, select the edges, choose a ball end mill, set the edge-break width, and generate the toolpath. Right-click the operation to edit it later.

![Creating a Ball Deburr toolpath in Fusion](docs/images/readme/ball-deburr-walkthrough.gif)

*Select edges, choose a tool and set the edge-break width.*

## How it works

Fusion Free Deburr generates custom 3D sketch geometry and uses Fusion's native Trace strategy to create the toolpath.

> **Alpha software:** Simulate and inspect the toolpath before machining.
