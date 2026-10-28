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

## Update 2 (reference render: faceted dome, LED, camera)
- Dome is now a mirror-symmetric low-poly polyhedron of planar triangles (`canopy_faceted`, `facet_vertices`, `facet_inset`) built through the measured levels.
- Front LED slit: a small slit through the front shell that opens into a pocket behind it, so an LED can be glued from inside. It is part of the one body (no separate LED part). Parameters `led_*` (sizes in final mm).
- Fixed front camera tilted down 20 deg under the nose (`camera_*`).
- Lower hull widened so it meets the guard rings (no air gap between body/arms and guards); arms are 16 x 12 mm.
- Previews: `preview_overview.png`, `preview_front.png`.

## Update 3 (size + LED slit)
- Overall footprint is 240 x 240 mm: square rotor layout (`motor_x == motor_z`) and `target_overall_mm` uniformly scales the model so the largest horizontal extent is 240 mm.
- The middle of the body is wider (`body_half_width`, `mid_widen*`), tail narrower.
- Separate `led_strip_green.*` files removed; the LED is only a slit + pocket in `drone_parametric.*`.
- Camera: much smaller; the module sits in a pocket inside the nose and only a small square housing (aligned with the lens axis, lens opening inside it) shows on the nose face. Lens axis points forward, tilted 35 deg down (`camera_*` parameters, final mm). The old external camera box was removed.
- Guards enlarged in place (`guard_grow` = 2.7 model units, wall thickness unchanged): outer radius ~48.9 mm final, 0.6 mm from the body at the closest point (not fused; arms still connect). The body keeps its size via the fixed `model_scale` (1.28548). Footprint is now ~247 x 247 mm (the 240 limit was lifted on request); `target_overall_mm` is None.
