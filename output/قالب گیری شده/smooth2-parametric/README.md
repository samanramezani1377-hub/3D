# smooth2-parametric

Parametric rebuild of the `smooth 2` quadcopter (CadQuery). Generator: `cad/parametric/drone_parametric.py`.

- Regenerate: `pip install cadquery && python3 cad/parametric/drone_parametric.py` (optionally `--params my.json`).
- Outputs: `drone_parametric.step`, `drone_parametric.stl`, `drone_parametric.params.json`, `validation-report.json`.
- Main parameters: `guard_outer_radius`, `guard_wall`, `guard_height`, `motor_x/motor_z`, `arm_width`, body/canopy dimensions, propeller size.
- The old lopsided guards (radius spread 7-13 mm) were replaced by true circular rings (outer radius spread ~0.01 mm).
- `smooth 2` source files were not modified. This is a clean re-construction from measurements, not a feature-exact copy; props are placeholders.

## Update (closer to the original)
- Canopy is lofted through measured sections of the original (`cad/parametric/body_sections.json`, produced by `cad/tools/extract_body_sections.py`; plain numbers you can edit).
- Guard rings are revolved from the measured barrel profile (outer radius 29 -> 35.3 -> 31 mm over the height), still exact circles.
- Propellers lowered to y=9 like the original, radius 25.5 mm.
- Check with `python3 cad/tools/validate.py` (volume, extents, surface distance to the original, ring roundness).
