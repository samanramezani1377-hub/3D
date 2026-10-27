# Quadcopter source mesh and CAD redesign (demo branch)

This branch is the work area for inspecting the original quadcopter and developing a separate modular redesign. The original geometry on main is treated as read-only. No source mesh or original CAD conversion output is overwritten.

## Source assets

- quadcopter_tidy.obj — original mesh
- quadcopter_tidy.stl — original STL
- output/quadcopter_tidy_complete.stl — preserved complete-mesh copy
- parts/ — separate body, duct, motor, and propeller mesh files

The source mesh is approximately 144.60 × 26.15 × 150.00 mm in its current coordinate orientation. These are source-mesh bounds, not verified printer-oriented dimensions.

## Workflows

1. **Convert original mesh to CAD** — fetches quadcopter_tidy.obj from main read-only and uploads the original faceted FreeCAD/STEP conversion and reports.
2. **Inspect original geometry** — reports mesh dimensions, triangle counts, watertightness, connected components, and existing part files.
3. **Build modular quadcopter redesign CAD** — runs cad/redesign/quadcopter_modular_redesign.FCMacro on demo and uploads a new parametric first-pass assembly, individual STEP/STL parts, and a design report.

## Modular redesign status

A first-pass parametric concept is now present under cad/redesign/. It includes:
- lower electronics tray with a nominally flat floor and underside M3 screw-head counterbores;
- upper lid and four underside clamp bosses;
- four independent arms, each with an integrated propeller guard and a provisional motor mounting pattern;
- a two-blade, serrated trailing-edge propeller concept informed by low-noise research;
- a fixed camera mount with a nominal 15-degree downward angle.

The initial values are editable in the FreeCAD macro. Motor mounting dimensions and shaft bore are placeholders, not measurements from the source mesh or a selected motor. Battery/electronics/camera fit and screw engagement are not verified. The action's XY bounds check does not replace slicer placement and orientation checks.

## Low-noise propeller design

Recent experimental work suggests that carefully tuned trailing-edge serrations can reduce tonal and broadband noise, but effects depend on geometry and RPM and may reduce thrust. Other work emphasizes reducing blade-tip loading and designing for the intended operating point. The generated propeller is only a research-informed prototype; no acoustic or thrust results are claimed. Research links and design caveats are documented in cad/redesign/README.md.

**Safety:** do not spin a printed prototype at flight RPM or fly the current concept. Validate material, balance, runout, vibration, thrust, current, temperature and sound on a guarded test stand. Use a commercially rated propeller matched to the motor for initial powered tests. This repository is not yet a flight-ready or safety-validated design.
