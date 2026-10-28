# Original quadcopter mesh → CAD

This repository contains the original quadcopter reference, mesh-derived CAD work, and a measured parametric reconstruction. All active changes must be made on branch `demo`; branch `main` is read-only.

## Current active working folder — authoritative

**The active working folder for the current quadcopter CAD task is:**

`output/قالب گیری شده/smooth2-parametric/`

Use this folder as the source of truth for the current working model and its generated exports. Do not treat the neighboring `output/قالب گیری شده/` exterior-shell candidate or the older `output/قالب گیری شده/smooth 2/` files as the active working target unless the user explicitly changes the decision.

Current files include:
- `drone_parametric.step` — generated STEP CAD export
- `drone_parametric.stl` — generated STL mesh export
- `drone_parametric.params.json` — parameter snapshot
- `validation-report.json` — existing validation report; currently stale relative to the latest regenerated STEP/STL and must not be presented as validation of those latest exports
- `preview_overview.png` — preview image
- `stage1-measurements.json` — measurement notes

## Python generator

The generator source is tracked separately at:

`cad/parametric/drone_parametric.py`

It uses measured body sections from `cad/parametric/body_sections.json` and writes generated outputs into `output/قالب گیری شده/smooth2-parametric/`. Keep the source script in its existing location; the output folder above remains the active CAD working folder.

The regeneration workflow is `.github/workflows/regenerate-smooth2-parametric.yml`. It runs only on `demo` and currently commits only the STEP and STL exports. Its checks verify that the files exist and are non-empty; they do not establish geometric validity, bounding-box correctness, or STL watertightness. Keep this limitation visible in project documentation until stronger validation is implemented.

## Current status and known limits

- This is a measured parametric reconstruction using CadQuery, not a feature-exact recovery of the original CAD design.
- The model includes a simplified two-blade propeller representation and clean primitive arms/motors; these are not exact reconstructions of all original mesh details.
- The latest code/export regeneration describes a target footprint of approximately 247 mm, but the checked-in validation report still records older extents around 144.66 × 28.53 × 153.67 mm. Measure the current STEP/STL before claiming the latest footprint is verified.
- The existing report says the STL is not watertight and notes about five open tessellation edges near the LED groove. This report is older than the latest STEP/STL exports, so the current STL must be checked again.
- Do not claim manufacturing readiness unless the current exports have actually passed the relevant geometry and mesh checks.

## Strict project rules

1. Continue work only on branch `demo`; never change branch `main`.
2. Use `output/قالب گیری شده/smooth2-parametric/` as the authoritative active working folder for this task.
3. Preserve existing files and make focused corrections; do not rebuild the model from scratch or replace existing geometry without a clear need.
4. Keep generated artifacts and reports consistent with the exact source/code revision that produced them.
5. Validate the current output files, not just old reports or successful file-existence checks.
