# Original quadcopter mesh → CAD

This branch (`demo`) is dedicated to improving the CAD files derived from the **original complete quadcopter model**. The source mesh `quadcopter_tidy.obj` on `main` is read-only and must never be modified.

## Current active work location — authoritative

**From now on, all CAD correction and cleanup work must be performed only on the extracted exterior-shell candidate in:**

`output/قالب گیری شده/`

This folder is the sole active working location for the next phase: identifying and removing incorrect/unwanted internal geometry while preserving the correct exterior shape.

Current files in that folder:
- `quadcopter_exterior_shell_candidate.FCStd` — native FreeCAD working file
- `quadcopter_exterior_shell_candidate.step` — STEP export of the extracted shell
- `exterior-shell-report.json` — extraction report
- `source-sha256.txt` — source fingerprint

Start from the existing `.FCStd` file in this folder. Save corrections and new working versions in this same folder, with clear filenames. Export corresponding STEP files there when needed.

## Strict working rules

- **Work only inside `output/قالب گیری شده/` for the active CAD-cleanup task.**
- Do not continue editing the previous canonical directory `output/original-quadcopter-mesh-cad/downloaded-cad/`; it is preserved as a historical/reference deliverable, not the active work target.
- Do not modify the source mesh `main/quadcopter_tidy.obj` or make any changes to branch `main`.
- Preserve the current files in `output/قالب گیری شده/` before making changes. Do not overwrite the starting candidate unless the user explicitly asks; prefer new, clearly named versions for corrections.
- Do not regenerate the CAD from the source mesh or rerun conversion workflows as a routine step.
- First inspect the actual existing candidate, identify demonstrably incorrect/unwanted internal geometry, and make focused corrections. Do not delete parts merely because they are internal; they may be legitimate structure.
- Validate each result after saving. Do not claim that internal geometry has been cleaned or that the model is manufacturing-ready unless the corresponding checks have actually passed.
- The current exterior-shell candidate is still mesh-derived/faceted. Extracting a shell does not by itself reconstruct smooth analytic/NURBS surfaces or remove all internal defects.

## Previous/reference deliverables

The earlier conversion files and reports remain preserved at:
`output/original-quadcopter-mesh-cad/downloaded-cad/`

Other historical conversion files may remain under `cad/original/` and `output/`; they are not the active work target.

## Workflow and source safety

- All project changes belong on `demo`.
- `main` is a read-only source and must remain unchanged.
- Do not add an unrelated redesign or extra workflow that changes the project's scope.
