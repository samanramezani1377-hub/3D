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
- **Each of the two tick branches must be 60% of the main arm width** (currently 16 mm arm width → 9.6 mm branch width before model scaling). This narrower branch width is intentional and does not mean the main arm beam itself should neck down.
- Mirror/rotate the geometry correctly for all four arm positions so the long branch direction follows the front/rear rule, rather than giving all roots the same orientation.
- Preserve the main arm's width through the body entry; do not neck down the arm at the point where it enters the body.
- **The arm's width and height must remain constant along its entire length.** Neither dimension may taper, shrink, or be locally reduced at the body-entry/root region or anywhere else along the arm.
- The tick-shaped root must not create a thinner floor/underside section or otherwise reduce the arm's original cross-section. Keep the full specified arm width and height through the transition into the body.
- Apply these constant-width and constant-height requirements to all four arms, including their mirrored/rotated roots.
- **Keep the arm's width and height constant along its full length.** Neither dimension may shrink or taper at the body entry, through the root, or anywhere else along the arm.
- The tick/check-mark root geometry must not reduce the arm's original cross-section: preserve both the original width and original height where the arm joins the body and throughout the transition.
- Apply these constant-width and constant-height requirements to all four arms, including their correctly mirrored/rotated roots.
- The intended mounting remains through the floor with vertical fasteners and underside nut recesses; do not change it to side-wall fastening.

**Implementation status: SOURCE UPDATED; REGENERATION/VALIDATION PENDING.** The generator now adds two integral, full-height branches to each arm root: a 16-unit longitudinal leg directed toward the fuselage centre (rear arms point forward; front arms point backward) and a 10-unit crosswise leg directed toward the centreline. Each branch is 60% of the main arm width. The main arm beam retains its full width and height. Body/crown clearance and the two vertical fastener bores are generated from the same tick-root geometry. The new source commit is `e03e45446ae8e5efc5e3d67a04af2544c20bfcf6`. Existing CAD exports/reports remain from an earlier generation until the triggered workflow completes; do not claim the new geometry is validated before then.

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
6. Describe the tick-root geometry as source-implemented but not validated until the workflow has regenerated and checked the exact new STEP/STL exports.
