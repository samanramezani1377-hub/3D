# Original quadcopter mesh → CAD

This branch (`demo`) is dedicated to improving the CAD files derived from the **original complete quadcopter model**. The source mesh `quadcopter_tidy.obj` on `main` is read-only and must never be modified.

## Source and current deliverables

- Source: `main/quadcopter_tidy.obj` (read-only)
- Preserved complete-mesh copy: `output/quadcopter_tidy_complete.stl`
- Native FreeCAD conversion: `cad/original/quadcopter_tidy_mesh_cad.FCStd`
- Conversion report: `cad/original/cad-conversion-report.json`
- Source inspection report: `cad/original/source-mesh-report.json`
- **Canonical existing final files and diagnostics:** `output/original-quadcopter-mesh-cad/downloaded-cad/`

## Important working rule: edit the existing final CAD files

The task is to **correct and improve the CAD deliverables that already exist** in `output/original-quadcopter-mesh-cad/downloaded-cad/`.

- Treat the files in that directory as the current final deliverables and preserve them while working.
- Inspect and modify the existing CAD model/file when a correction is needed.
- **Do not regenerate the CAD from the source mesh, rebuild the output package, or replace the existing final files** as a routine step.
- Do not run repeated comparisons against earlier versions or spend time re-reporting differences unless a specific comparison is necessary to diagnose a concrete defect.
- Do not create a conceptual redesign or substitute a newly invented model for the actual quadcopter.
- If a change genuinely requires replacing a final file, do so only when explicitly requested; keep the change focused and explain exactly which file was changed.
- Do not claim a CAD correction is complete until the edited file has actually been saved and the correction has been checked.

## Workflow and source safety

The conversion workflow is available in GitHub Actions, but a conversion run is **not** the default next step for CAD correction. Work on the existing deliverables first; do not use the workflow to regenerate or replace them unless explicitly requested.

- All project changes belong on `demo`.
- `main` is a read-only source and must remain unchanged.
- Do not add an unrelated redesign or extra workflow that changes the project's scope.

## What this conversion means

The existing CAD originated from the original triangular mesh and therefore retains faceted geometry. It is not a fully reconstructed parametric mechanical design, and smooth analytic/NURBS surfaces have not automatically been rebuilt. Make only corrections grounded in the actual model and the specific defect being addressed; do not label it manufacturing-ready without appropriate checks.
