# Modular guarded quadcopter — demo branch

This branch contains a parametric OpenSCAD concept for a modular guarded quadcopter. Original OBJ/STL assets on the branch are retained.

## Design intent
- Two-piece body: upper cover and lower tray with a flat electronics floor.
- Four separate arms sandwiched between the body halves.
- Four underside M3-class screw clearances (nominal 3.2 mm) at body corners.
- Separate circular propeller guard module for each arm.
- Motor mounting area at each arm tip (generic placeholder pattern).
- Fixed camera bracket with approximately 18-degree downward tilt.
- Nominal frame layout about 240 mm; the body and each module are intended to fit a 200 × 200 mm bed, but every exported STL's bounding box must be checked in the slicer.

## CAD file
- `cad/quadcopter_demo.scad`: parametric geometry. Set `PART` near the top to `top`, `bottom`, `arm`, `rotor_guard`, `propeller`, or `camera_mount`. `assembly` previews the arrangement.

### Exporting parts
1. Open the SCAD file in OpenSCAD.
2. Set `PART` to the component you want.
3. Press F6 (Render), then export as STL.
4. Duplicate arms and guards four times. Check orientation, supports, clearances and the actual bounding box in your slicer before printing.

## Printable propeller prototype
The SCAD includes a complete, printable three-blade planform, not just a hub. It is a **geometry prototype only**: blade pitch, airfoil, motor/shaft fit, material, print orientation, fatigue strength, thrust, efficiency, noise and dynamic balance have not been validated. Do not run it at flight RPM or fly it without professional engineering validation and controlled guarded testing. A blade can fracture and cause serious injury. For initial powered testing, a matched commercial propeller is safer.

## Low-noise design notes
Research indicates that trailing-edge serrations can reduce broadband and tonal noise in some conditions, but results are geometry- and RPM-dependent and may reduce thrust. See `docs/quiet-propellers.md` for papers and test recommendations. Noise comparisons should be made at equal thrust and measured RPM, with a fixed microphone distance and direction.

## Engineering status and missing inputs
This is not yet a flight-ready airframe or a verified print package. The SCAD has not been validated by a mesh repair/manifold check or by physical test. Generic dimensions were assumed because the exact motor and bolt pattern, propeller size, battery, ESC, flight controller, camera, target thrust, filament and printer tolerances were not supplied. Before manufacture, validate arm clamping, screw lengths, motor mounts, guard-to-prop clearance, center of gravity, camera field of view, battery retention, structural loads, vibration, heat, electrical safety, and each part's bounding box. The design should be iterated against actual components before flight.