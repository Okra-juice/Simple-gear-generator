# Simple Gear Generator

A parametric gear generation tool written in C++ (pure Win32 API). Configure parameters to export 3D-printable STL files or CAD-importable STEP files, with a **built-in real-time 3D preview** and **bilingual (Chinese/English) interface**. No third-party dependencies required.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Gear Types](#gear-types)
- [Parameter Reference](#parameter-reference)
- [Center Hole Feature](#center-hole-feature)
- [Chamfer Feature](#chamfer-feature)
- [Center Boss](#center-boss)
- [Stacked Gear (Multi-Layer)](#stacked-gear-multi-layer)
- [Layer Edit Dialog](#layer-edit-dialog)
- [3D Real-Time Preview](#3d-real-time-preview)
- [Language Switching](#language-switching)
- [Export Formats & Units](#export-formats--units)
- [Building from Source](#building-from-source)
- [Validation Rules](#validation-rules)
- [Notes & Caveats](#notes--caveats)

---

## Quick Start

1. **Double-click** `SimpleGearGenerator.exe` to run (no installation required, single-file executable).
2. Select **Language** (中文 / English) and **Gear Type** from the top of the window.
3. Fill in the parameters in the left panel (all dimensions are in millimeters).
4. The right-side 3D preview updates **in real time** as you type — drag to rotate, scroll to zoom.
5. Once satisfied, select the **Export Format** (STL / STEP) and **Output Unit** at the bottom.
6. Click the **Generate** button and choose a save path in the dialog.
7. Open STL files in slicing software (Cura, PrusaSlicer, etc.) for 3D printing; open STEP files in CAD software (SolidWorks, FreeCAD, etc.) for further editing.

---

## Gear Types

The tool supports **5 gear types**, selectable from the dropdown at the top of the window:

### 1. Spur Gear

Standard involute spur gear — the most common gear type.

- **Tooth profile**: Standard involute, 20° pressure angle
- **Module calculation**: `module = outerDiameter / (teeth + 2)`
- **Root circle**: `root diameter = module × (teeth - 2.5)` (1.25× dedendum clearance)
- **Profile sampling**: Each tooth is built from a half-profile rotated + mirrored, containing 4 tip points + 12 involute flank points + 3 root points

**Available parameters**: Outer diameter, teeth, thickness, center boss, center hole, chamfer

### 2. Crowned Gear (Barrel)

Barrel/drum-shaped gear with teeth that bulge radially at the mid-plane, used to compensate for shaft misalignment.

- **Profile**: Based on standard involute, with radial bulge along the thickness axis
- **Barrel amount**: Radial expansion of the mid-section relative to the ends
- **Constraint**: Barrel amount must be less than `1.1 × module`

**Available parameters**: Outer diameter, teeth, thickness, barrel amount, center boss, center hole

### 3. Crown Wheel (Face Gear)

A thin disc with a ring of trapezoidal teeth standing up along the axial direction, shaped like a crown. Can mesh with a spur gear at a 90° axis angle.

- **Structure**: Bottom is a thin disc; teeth rise along the outer edge of the disc
- **Sheet thickness**: Thickness of the disc portion
- **Tooth height**: Axial height of the standing teeth

**Available parameters**: Outer diameter, teeth, sheet thickness, tooth height, center boss, center hole

### 4. Internal Gear

Ring-shaped gear with teeth on the inner circumference. Can mesh with an external spur gear to form planetary gear mechanisms.

- **Generation method**: Builds a standard external tooth contour, then reflects it through the pitch circle (`ρ → 2r - ρ`), making teeth point inward
- **Module calculation**: `module = outerDiameter / (teeth + 6.5)` (different formula from external gears)
- **Structure**: Outer boundary is a smooth circle; inner boundary has the toothed profile
- **"Outer diameter" meaning**: Refers to the ring's outer diameter, not the tip circle diameter

**Available parameters**: Outer diameter (ring OD), teeth, thickness, chamfer
> Note: Internal gears do not support center boss or center hole (the center is already empty).

### 5. Stacked Gear (Multi-Layer)

Stacks multiple gear layers of different types/parameters along the Z-axis into a single part — e.g., planetary gear systems or compound gears.

- **Layer count**: No upper limit (minimum 1 layer)
- **Independent per layer**: Each layer can have its own gear type, outer diameter, teeth, thickness, etc.
- **Height accumulation**: Layers stack along Z, total height = sum of all layer heights
- **Supported sub-types**: Spur, crowned, crown wheel, internal (nested stacking is not supported)

**How to use**: Selecting this type switches the left panel to a layer management interface with a list view and buttons.

---

## Parameter Reference

### Basic Parameters

| Parameter | Unit | Applies To | Description |
|-----------|------|------------|-------------|
| Outer Diameter | mm | All | Tip circle diameter (spur/crowned); ring outer diameter (internal); tooth ring outer diameter (crown wheel) |
| Teeth | — | All | Number of teeth, minimum 5 |
| Thickness | mm | Spur / Crowned / Internal | Gear body thickness |
| Barrel Amount | mm | Crowned | Radial expansion at mid-section vs. ends; must be < 1.1×module |
| Sheet Thickness | mm | Crown Wheel | Thickness of the disc portion |
| Tooth Height | mm | Crown Wheel | Axial height of standing teeth |

### Module & Tooth Profile

- **Pressure angle**: Fixed at 20° (standard value), not adjustable
- **Involute solving**: Bisection method (80 iterations) to solve the involute function `inv(α) = tan(α) - α`
- **Base circle radius**: `rb = r × cos(20°)`
- **Dedendum clearance**: 1.25× module (standard value)
- **Module display**: After successful generation, the log shows the auto-calculated module (e.g., `Module=2.000mm`)

---

## Center Hole Feature

Creates a through-hole at the center of the gear for mounting on a shaft. Enable by checking "Add center hole".

### Hole Diameter

- Circle: The hole's diameter
- Square: The square's side length
- Polygon: The circumscribed circle diameter
- Star: The outer circumscribed diameter (inner radius = 50% of outer)
- D-shape: Defined by separate "Length" and "Width" parameters (see below)

### Hole Shape

When "Custom hole shape" is **unchecked**, the hole is a **circle** by default. When checked, you can choose from 5 shapes:

| Shape | Description | Extra Parameters |
|-------|-------------|-----------------|
| Circle | Standard round hole, 128-segment approximation | Diameter |
| D-shape | Round hole with a flat side (D-shaped cross-section) | Length, Width, optional ratio lock |
| Square | Square hole, side length = hole diameter | Diameter |
| Polygon | Regular polygon hole (3+ sides) | Circumscribed diameter, number of sides (≥3, default 6) |
| Star | Star-shaped cross-section hole | Outer diameter, number of points (≥3, default 5) |

### D-shape Parameters

- **Length**: Full length of the D-shaped cross-section
- **Width**: Width of the D-shaped cross-section
- **Lock ratio**: When checked, modifying length or width automatically maintains the ratio (width/length ≈ 0.8)
- **Constraint**: Must satisfy `half length < width < length`

### Hole Size Limits

The hole's maximum circumscribed diameter must be smaller than the limiting diameter:
- With boss: limiting diameter = boss diameter
- Without boss: limiting diameter = root circle diameter
- Crown wheel: limiting diameter = outer diameter × 0.62

---

## Chamfer Feature

Creates beveled edges on the gear to reduce stress concentration and burrs. Only available for **Spur Gear** and **Internal Gear** types.

Enable by checking the "Chamfer" checkbox, then set the chamfer size and location.

### Chamfer Location

| Option | Description |
|--------|-------------|
| None | No chamfer (even with chamfer checked, selecting "None" = no chamfer) |
| Top edge | Chamfer at the top face edge of the gear |
| Bottom edge | Chamfer at the bottom face edge of the gear |
| Top & bottom | Chamfer on both top and bottom face edges |
| Tooth tips | Chamfer at each tooth tip (reduces tip circle radius) |

> Note: Internal gears do not support "Tooth tips" chamfer — only top/bottom/both.

### Chamfer Size

- The actual chamfer height is auto-clamped: `chamfH = min(chamferSize, thickness × 0.4)` to prevent over-chamfering
- Chamfer size must be > 0

### How It Works

- **Top/Bottom edge**: Scales the contour by factor `((tipRadius - chamferSize) / tipRadius)` and builds a tapered wall between the full and reduced contours
- **Tooth tips**: Scales the entire outer contour inward by the chamfer size (reduces tip circle radius)
- **Internal gear**: Both the outer ring and inner tooth contour are scaled (outer ring shrinks inward, inner contour moves toward center)

---

## Center Boss

Adds a raised cylindrical hub at the center of the gear's top face, for mounting or positioning. Enable by checking "Add center boss".

| Parameter | Description |
|-----------|-------------|
| Boss Thickness | Height of the boss (rises above the gear's top face) |
| Boss Diameter | Diameter of the boss cylinder |

### Boss Constraints

- Boss diameter must be < the limiting diameter:
  - Spur gear: limiting diameter = root circle diameter
  - Crowned gear: limiting diameter = root circle diameter - 2 × barrel amount
  - Crown wheel: limiting diameter = outer diameter × 0.62
- Boss thickness must be > 0
- If a center hole is also present, the boss is annular (the hole passes through it)

> Note: Internal gears do not support center boss.

---

## Stacked Gear (Multi-Layer)

Selecting "Stacked Gear" as the gear type switches the left panel to a layer management interface.

### Layer Management Controls

| Control | Description |
|---------|-------------|
| Layer Count | Displays the current total number of layers; directly editable |
| Add Layer | Adds a default spur gear layer at the end (OD 40mm, 20 teeth, 5mm thick) |
| Remove Layer | Removes the currently selected layer from the list |
| Layer List | 5-column table: # / Type / Outer Dia / Teeth / Height |

### Default Layer Configuration

Initially has 2 layers:
- Layer 1: Spur gear, OD 40mm, 20 teeth, 5mm thick, boss 5mm×20mm
- Layer 2: Spur gear, OD 30mm, 15 teeth, 4mm thick

### Stacking Height Calculation

Each layer's height = that layer's gear thickness + that layer's boss thickness (if any). Layers stack along Z, accumulating height.

### Supported Sub-Types

Each layer can be set to one of 4 types: Spur, Crowned, Crown Wheel, or Internal (nested stacking is not supported).

---

## Layer Edit Dialog

**Double-click a row** in the layer list to open the layer edit dialog, where you can configure that layer's parameters in detail.

### Dialog Controls

The dialog contains the following controls, which dynamically show/hide based on the selected gear type:

| Control | Applies To | Description |
|---------|------------|-------------|
| Gear Type | All | Dropdown (Spur / Crowned / Crown Wheel / Internal) |
| Outer Diameter | All | Gear outer diameter |
| Teeth | All | Number of teeth (≥5) |
| Thickness | Spur / Crowned / Internal | Body thickness |
| Sheet Thickness | Crown Wheel | Disc thickness |
| Tooth Height | Crown Wheel | Axial tooth height |
| Barrel Amount | Crowned | Radial expansion amount |
| Add Center Boss | Spur / Crowned / Crown Wheel | Checkbox; when checked, shows boss thickness + boss diameter |
| Add Center Hole | Spur / Crowned / Crown Wheel | Checkbox + hole diameter |
| Hole Shape | Spur / Crowned / Crown Wheel | Dropdown (Circle / D-shape / Square / Polygon / Star) |
| Chamfer | Spur / Internal | Checkbox; when checked, shows chamfer size + chamfer location |
| OK / Cancel | — | Confirm or discard changes |

### Dynamic Visibility Rules

- **Internal gear**: Hides boss and center hole controls; shows chamfer controls
- **Crown wheel**: Hides thickness; shows sheet thickness + tooth height
- **Crowned**: Shows barrel amount
- **Chamfer** controls only appear for Spur and Internal gear types

---

## 3D Real-Time Preview

The right-side 3D preview uses a pure software renderer (no GPU required) with the following interactions:

### Controls

| Action | Effect |
|--------|--------|
| Left-click drag | Rotate view (yaw + pitch); pitch clamped to ±1.45 radians |
| Mouse wheel | Zoom (×1.15 per notch; range 0.1–20.0) |

### Rendering Characteristics

- **Projection**: Orthographic, centered on the mesh centroid
- **Lighting**: Lambertian model, light direction (0.45, -0.55, 0.7)
- **Shading**: 33-level grayscale, base color RGB(182, 188, 198)
- **Back-face culling**: Invisible faces are not rendered
- **Depth sorting**: Painter's algorithm (sorted by average Y value per triangle)
- **Double buffering**: Offscreen DC + bitmap to prevent flickering
- **Background**: Light gray RGB(246, 248, 251)

### Preview Updates

- Any parameter change (text input `EN_CHANGE` event) triggers mesh rebuild and redraw
- Switching gear type or toggling boss/hole/shape/chamfer checkboxes also triggers relayout + rebuild

---

## Language Switching

The language dropdown at the top of the window supports instant switching between two languages:

| Option | Description |
|--------|-------------|
| 中文 | All interface text switches to Chinese (default) |
| English | All interface text switches to English |

Switching language immediately updates all control labels, dropdown options, and list column headers — without affecting already-entered parameter values.

> Note: The log window output is always in English (even when the UI is set to Chinese), which does not affect usage.

---

## Export Formats & Units

### Export Formats

| Format | Default Filename | Filter | Use Case |
|--------|-------------------|--------|----------|
| STL (binary) | gear.stl | *.stl | 3D printing (slicing software) |
| STEP (AP214) | gear.step | *.step | CAD import/editing (SolidWorks, FreeCAD, etc.) |

- **STL format**: Standard binary STL — 80-byte header + triangle count + per-triangle (normal + 3 vertices + attribute)
- **STEP format**: Full STEP AP214 (ISO 10303-21) — each triangle becomes an ADVANCED_FACE → EDGE_LOOP → VERTEX, assembled into CLOSED_SHELL → MANIFOLD_SOLID_BREP. Units set to millimeters.

### Output Units

| Option | Scale Factor | Description |
|--------|-------------|-------------|
| Millimeter | 1.0 | Default, suitable for most 3D printing |
| Centimeter | 0.1 | Export coordinates ×0.1 |
| Inch | 1/25.4 | Export coordinates ÷25.4 |
| Meter | 0.001 | Export coordinates ×0.001 |

The UI input is always in millimeters; only the exported file coordinates are scaled by the selected unit.

### Generation Process (6 Steps)

After clicking the "Generate" button, the log window shows 6-step progress:

1. **[1/6] Read parameters**: Reads all parameters from the UI (or from the layer list for stacked mode)
2. **[2/6] Validate**: Checks all parameter constraints; aborts on failure
3. **[3/6] Build mesh**: Computes involute tooth profile and constructs the 3D triangle mesh
4. **[4/6] Prepare export**: Determines format, unit, and scale
5. **[5/6] Select output file**: Opens a save dialog to choose the file path
6. **[6/6] Write file**: Writes STL or STEP; logs success or failure

---

## Building from Source

The source is a single file `SimpleGearGenerator.cpp` + resource file `app.rc` + icon `app.ico`, with no third-party dependencies — only the Windows SDK is needed.

### Visual Studio / MSVC

```bash
rc /nologo /fo app.res app.rc
cl /EHsc /O2 /utf-8 SimpleGearGenerator.cpp app.res /link /SUBSYSTEM:WINDOWS /OUT:SimpleGearGenerator.exe
```

- `/EHsc`: C++ exception handling
- `/O2`: Speed optimization
- `/utf-8`: Source code is UTF-8 encoded
- `/SUBSYSTEM:WINDOWS`: Window subsystem (no console popup)

### MinGW (g++)

```bash
windres app.rc -o app.res
g++ -O2 SimpleGearGenerator.cpp app.res -o SimpleGearGenerator.exe -mwindows -lcomctl32 -lcomdlg32 -luser32 -lgdi32
```

- `-mwindows`: Windows GUI application
- `-lcomctl32`: Common controls library (ComboBox, ListView, etc.)
- `-lcomdlg32`: Common dialogs library (file save dialog)
- `-luser32` / `-lgdi32`: User interface + Graphics Device Interface

---

## Validation Rules

All parameters are automatically validated before generation. Invalid parameters cause an error and abort:

| Check | Rule | Error Message |
|-------|------|---------------|
| Outer diameter | > 0 | Outer diameter must be > 0 |
| Teeth | ≥ 5 | Teeth must be at least 5 |
| Thickness | > 0 | Thickness must be > 0 |
| Barrel amount | > 0 and < 1.1×module | Barrel amount too large: must be < 1.1×module |
| Sheet thickness | > 0 | Sheet thickness must be > 0 |
| Tooth height | > 0 | Tooth height must be > 0 |
| Boss thickness | > 0 | Boss thickness must be > 0 |
| Boss diameter | > 0 and < limiting diameter | Boss diameter must be smaller than max |
| Hole diameter | > 0 | Hole diameter must be > 0 |
| D-shape length/width | > 0 and half length < width < length | half length < width < length |
| Polygon sides | ≥ 3 | Polygon sides must be at least 3 |
| Star points | ≥ 3 | Star points must be at least 3 |
| Hole size | Max circumscribed diameter < limiting diameter | Hole too large |
| Chamfer size | > 0 (when chamfer is enabled) | Chamfer size must be > 0 |
| Stacked layers | ≥ 1 | Add at least one gear layer |

---

## Notes & Caveats

1. **Root transition**: Uses a simplified straight-line approach (not an exact root fillet), which has negligible impact on 3D printing accuracy.
2. **Boss direction**: The boss is on one side of the gear (top face). For symmetric bosses on both sides, either mirror-and-stitch in your slicer after export, or modify the boss generation logic in the source code.
3. **STL units**: STL files do not store units; slicing/CAD software interprets coordinates using its own default unit. If your software's default unit is not millimeters, select the matching unit in "Output Unit" (e.g., choose "Meter" for Blender, "Inch" for inch-based CAD).
4. **Internal gear limitations**: Does not support center boss or center hole (the center is already empty).
5. **Internal gear chamfer**: Does not support "Tooth tips" position — only top edge / bottom edge / top & bottom.
6. **Stacking limitation**: Nested stacking (a stacked gear inside a stacked layer) is not supported.
7. **Real-time preview**: The preview uses software rendering (CPU). Complex gears (many teeth + many stacked layers) may cause slight lag — this is normal.
8. **Log language**: The log window output is always in English, even when the UI language is set to Chinese. This does not affect parameter configuration or file generation.
