# smooth2-parametric

Parametric rebuild of the `smooth 2` quadcopter (CadQuery). Generator: `cad/parametric/drone_parametric.py`.

- Regenerate: `pip install cadquery && python3 cad/parametric/drone_parametric.py` (optionally `--params my.json`).
- Outputs: `drone_parametric.step`, `drone_parametric.stl`, `drone_parametric.params.json`, `validation-report.json`.
- Main parameters: `guard_outer_radius`, `guard_wall`, `guard_height`, `motor_x/motor_z`, `arm_width`, body/canopy dimensions, propeller size.
- The old lopsided guards (radius spread 7-13 mm) were replaced by true circular rings (outer radius spread ~0.01 mm).
- `smooth 2` source files were not modified. This is a clean re-construction from measurements, not a feature-exact copy; props are placeholders.
