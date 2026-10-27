# Original quadcopter mesh → CAD

This branch (`demo`) is dedicated to converting the **original complete quadcopter mesh** into CAD-derived files. The source mesh `quadcopter_tidy.obj` on `main` is read-only and must never be modified.

## Source and preserved geometry

- Source: `main/quadcopter_tidy.obj` (read-only)
- Preserved complete-mesh copy: `output/quadcopter_tidy_complete.stl`
- Native FreeCAD conversion: `cad/original/quadcopter_tidy_mesh_cad.FCStd`
- Conversion report: `cad/original/cad-conversion-report.json`
- Source inspection report: `cad/original/source-mesh-report.json`
- Downloaded/exported conversion files and diagnostics: `output/original-quadcopter-mesh-cad/downloaded-cad/`

## Workflow

Only one workflow is retained: **Convert original mesh to editable CAD**. It is manually triggered from GitHub Actions and reads the source mesh from `main` without writing to that branch.

Open **Actions → Convert original quadcopter mesh to editable CAD → Run workflow** on the `demo` branch.

## What this conversion means

The current CAD is derived from the original triangular mesh and preserves its faceted geometry. It is **not** a reconstructed parametric mechanical design, and smooth analytic/NURBS surfaces have not been rebuilt. A closed-solid candidate, when available, is still mesh-derived and must be checked against the validation and geometry-comparison reports before being treated as manufacturing-ready.

No separate conceptual redesign is part of this project. The task is to continue validating and improving the conversion of the actual source model.

## Branch safety

- All project changes belong on `demo`.
- `main` is a read-only source and must remain unchanged.
