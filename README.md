# Original quadcopter mesh → CAD

This repository contains the original quadcopter reference, mesh-derived CAD work, and a measured parametric reconstruction. All active changes must be made on branch `demo`; branch `main` is read-only.

## Current active working folder — authoritative

**The active working folder for the current quadcopter CAD task is:**

`output/قالب گیری شده/smooth2-parametric/`

Use this folder as the source of truth for the current working model and its generated exports. Do not treat the neighboring `output/قالب گیری شده/` exterior-shell candidate or the older `output/قالب گیری شده/smooth 2/` files as the active working target unless the user explicitly changes the decision.

Current files include:
- `drone_parametric.step` — existing generated STEP CAD export
- `drone_parametric.stl` — existing generated STL mesh export
- `drone_parametric.params.json` — parameter snapshot
- `validation-report.json` — existing report; do not treat it as validation of any ungenerated future revision
- `preview_overview.png` — preview image
- `stage1-measurements.json` — measurement notes

## Python generator

The generator source is tracked separately at:

`cad/parametric/drone_parametric.py`

It uses measured body sections from `cad/parametric/body_sections.json` and writes generated outputs into `output/قالب گیری شده/smooth2-parametric/`. Keep the source script in its existing location; the output folder above remains the active CAD working folder.

The regeneration workflow is `.github/workflows/regenerate-smooth2-parametric.yml`. Its checks verify that expected files exist and are non-empty, and include metadata/report assertions; these checks do not independently establish STEP round-trip validity or STL watertightness. Keep this limitation visible until stronger validation is implemented.

## Arm-root / body-entry geometry — required design

The arm root where each arm enters the fuselage must look like a **tick/check mark (✓-like)**, not a symmetric V and not a T.

- The root has two unequal branches that meet at the arm's inner junction: one long branch and one short branch.
- The **long branch must run along the aircraft's longitudinal direction**. It points **forward on the rear arms** and **backward on the front arms**.
- The short branch is the crosswise leg, creating the angled tick/check-mark appearance.
- Mirror/rotate the geometry correctly for all four arm positions so the long branch direction follows the front/rear rule, rather than giving all roots the same orientation.
- Preserve the main arm's width through the body entry; do not neck down the arm at the point where it enters the body.
- The intended mounting remains through the floor with vertical fasteners and underside nut recesses; do not change it to side-wall fastening.

**Implementation status: NOT IMPLEMENTED / NOT RUN.** This section records the required design intent only. The generator has not yet been updated and run to implement this tick-shaped, directionally oriented root. Existing CAD exports and validation reports therefore must not be described as containing or verifying this requested geometry. After implementation, regenerate the model on `demo`, inspect all four roots, and validate the newly generated STEP/STL before claiming completion.

## Current status and known limits

- This is a measured parametric reconstruction using CadQuery, not a feature-exact recovery of the original CAD design.
- The model includes a simplified two-blade propeller representation and clean primitive arms/motors; these are not exact reconstructions of all original mesh details.
- The latest recorded assembly report gives a bounding box of approximately 247.08 × 247.03 × 39.98 mm. This is a recorded result, not proof that a future modified revision has been generated or checked.
- The report records in-memory CadQuery shapes as valid, but lists STL watertightness and STEP round-trip import as `not_checked`.
- The report records `physical_fit_tested: false`; printed fit, tool access, and flight-load strength remain untested.
- Screw and nut solids are simplified envelopes without threads; use real metal M3 hardware for a physical build.
- Do not claim manufacturing readiness unless the current exports have actually passed the relevant geometry and mesh checks.

## Strict project rules

1. Continue work only on branch `demo`; never change branch `main`.
2. Use `output/قالب گیری شده/smooth2-parametric/` as the authoritative active working folder for this task.
3. Preserve existing files and make focused corrections; do not rebuild the model from scratch or replace existing geometry without a clear need.
4. Keep generated artifacts and reports consistent with the exact source/code revision that produced them.
5. Validate the current output files, not just old reports or successful file-existence checks.
6. Until the tick-shaped arm-root geometry is implemented and regenerated, describe it as a requirement only, not as completed work.
