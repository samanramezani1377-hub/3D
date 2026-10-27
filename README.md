# Quadcopter source mesh and CAD conversion (demo branch)

This branch is the work area for inspecting the original quadcopter and preparing it for a careful redesign. The original geometry on `main` is treated as read-only.

## Source assets

- `quadcopter_tidy.obj` — original mesh
- `quadcopter_tidy.stl` — original STL
- `output/quadcopter_tidy_complete.stl` — preserved complete-mesh copy
- `parts/` — separate body, duct, motor, and propeller mesh files

The source mesh is approximately 144.60 × 26.15 × 150.00 mm in its current coordinate orientation. These are mesh bounds, not verified printer-oriented dimensions.

## Workflows

Workflows are manual-only to avoid reinstalling tools or reconverting the mesh on every commit.

1. **Convert original mesh to CAD** — GitHub Actions → “Convert original quadcopter mesh to editable CAD” → Run workflow. It fetches `quadcopter_tidy.obj` from `main` read-only and uploads a downloadable artifact containing a FreeCAD document, STEP export, and reports.
2. **Inspect original geometry** — GitHub Actions → “Inspect original quadcopter geometry” → Run workflow. It reports mesh dimensions, triangle counts, watertightness, connected components, and checks the existing part files.

The generated CAD is mesh-derived faceted geometry. Conversion does not recover the original design history, smooth analytic surfaces, or editable parametric features. A closed-solid candidate, if produced, is still faceted and must be inspected before use.

## Redesign status

A separate parametric redesign is **not yet implemented**. The intended direction is to preserve the original quadcopter as the reference and then design/test separate upper shell, lower electronics tray, four clamped arms, underside fasteners, arm-mounted guards and motor interfaces, plus a fixed downward-angled camera mount.

Before printing, actual motor/propeller dimensions, screw pattern and lengths, battery/electronics sizes, camera field of view, material, printer tolerances, and required loads must be confirmed. Each individual exported part must be checked against the 200 × 200 mm print bed in the slicer. Do not treat this repository as a flight-ready or safety-validated design.

## Propellers and noise

No custom quiet propeller is currently validated here. Noise and thrust depend on propeller geometry, RPM, motor, and operating point. Use a commercially matched propeller for initial powered tests; do not spin an unvalidated printed propeller at flight RPM.
