# Modular guarded quadcopter — demo branch

This branch adds a **parametric OpenSCAD concept** based on the requested layout. The original OBJ/STL files are retained; this is a new design proposal, not a claim that the existing mesh was automatically remodeled.

## Design intent
- Two-piece body: upper cover and lower tray.
- Four separate arms intended to be clamped between the body halves.
- Four underside screw locations (nominal 3.2 mm clearance for M3-class hardware).
- Separate circular propeller guard module positioned at each arm tip.
- Flat internal floor for battery and flight electronics.
- Fixed camera mount tilted approximately 18 degrees downward.
- Nominal overall layout about 240 mm; individual modules are intended to fit a 200 × 200 mm print bed.

## Files
- `cad/quadcopter_demo.scad`: parametric concept. Set `PART` to `top`, `bottom`, `arm`, `rotor_guard`, `propeller`, or `camera_mount`.
- `docs/quiet-propellers.md`: research notes on current low-noise propeller work.

## Export
Open the SCAD file in OpenSCAD, set `PART` to a component, press F6 (Render), then export STL. Duplicate the arm and guard four times for assembly. Verify actual bounding boxes and all hardware clearances before printing.

## Important limits
This is a **concept prototype, not a flight-ready airframe**. Geometry has not been validated for structural loads, vibration, airflow, thermal management, battery retention, crash safety or watertight/manifold meshes. The SCAD propeller is only a visual placeholder and must not be used for flight. Use commercially manufactured, correctly matched and dynamically balanced propellers for powered tests; printed rotors can fail catastrophically at RPM.

Motor model and bolt pattern, battery/ESC/flight-controller dimensions, camera model, target thrust and printer tolerances were not supplied. These must be checked before manufacturing. The current concept should be reviewed and mechanically tested before flight.