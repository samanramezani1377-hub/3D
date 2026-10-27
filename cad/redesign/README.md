# Modular quadcopter redesign — first parametric prototype

This folder introduces a **new parametric concept**. It does not overwrite the original mesh or the earlier mesh-derived STEP files.

## Included design

- **quadcopter_modular_redesign.FCMacro** — FreeCAD macro that builds a native assembly and exports separate STEP/STL files.
- **.github/workflows/build-modular-redesign.yml** — runs the macro on the demo branch and uploads the generated CAD package as a GitHub Actions artifact.

The output package contains a native FreeCAD assembly, an assembly STEP, and separate printable exports for:
- lower electronics tray with a flat floor and underside screw-head counterbores;
- upper lid with four underside bosses;
- four identical independent arms, each with an integrated circular propeller guard;
- four instances of the low-noise propeller concept;
- fixed camera plate tilted down by 15 degrees.

The arm-root clamp uses four M3 clearance holes, one per arm. Each screw passes from the underside through the tray floor and arm root into the upper lid's boss. This is a conceptual clamping stack; the final screw length, insert/thread engagement, and local wall strength must be checked against the chosen hardware.

## Current nominal dimensions (editable in the macro)

| Feature | First-pass value |
|---|---:|
| Lower tray outer size | 72 × 60 mm |
| Lower tray height / floor | 18 / 3 mm |
| Upper lid thickness | 5 mm |
| Propeller radius | 19 mm |
| Guard outer diameter | 47 mm |
| Guard inner diameter | 41 mm |
| Camera downward angle | 15° |
| Print-bed XY limit | 200 × 200 mm |

The motor mount pattern (9 mm square, 2.2 mm holes) and shaft bore (2 mm) are **provisional placeholders**, not measurements from the existing mesh. The battery, flight controller, ESC, wiring, camera dimensions, motor pattern, propeller shaft, and required thrust have not been provided, so this is not yet a verified fit to the actual hardware.

## Low-noise propeller research

1. **Candeloro, Ragni & Pagliaroli (2026), “Experimental investigation on the application of serrated trailing edge propellers for drone noise reduction,” Applied Acoustics.** The study reports potential reductions around 3–4 dB with carefully tuned trailing-edge serrations, but also reports an aerodynamic trade-off, including nearly 20% lower thrust coefficient in advanced flight for the tested design. Results are specific to the tested geometry and operating conditions. https://doi.org/10.1016/j.apacoust.2025.111045
2. **Santamaria et al. (2026), “Serration manufacturing effects on propeller trailing edge noise mechanisms,” Journal of Sound and Vibration.** Experiments on 3D-printed propellers found that serration manufacturing details matter and that acoustic benefits decrease as rotational speed increases. https://doi.org/10.1016/j.jsv.2026.119656
3. **Santamaria et al. (2025), “Shape considerations for the design of propellers with trailing edge serrations,” Journal of Sound and Vibration.** The study discusses how serration geometry should depend on blade-tip Mach number and reports that a square-wave serration can outperform conventional sawtooth serration for small propellers. https://doi.org/10.1016/j.jsv.2024.118771
4. **Ghoreyshi et al. (2023), “Computational investigation of quiet propeller designs for small unmanned aerial vehicles,” Aerospace Science and Technology.** The study evaluates reduced loading near the blade tip as a noise-control strategy. https://doi.org/10.1016/j.ast.2023.108351

### Design interpretation

The macro uses a two-blade concept with a reduced chord and twist toward the tip, a rounded final tip station, and modest trailing-edge serrations. This is a literature-informed *starting point*, not a claim that the generated propeller is quieter. A propeller's noise depends strongly on diameter, RPM, thrust loading, motor, blade geometry, and installation. Lower RPM at the same required thrust and reduced tip loading are important design levers, but a larger propeller may require a different frame and motor.

### Safety and validation gates

**Do not spin the printed propeller at flight RPM or fly this design as-is.** Printed rotating blades can fail from layer adhesion, imbalance, defects, or resonance. Before any powered test, use a guarded test stand and validate dimensions, shaft fit, static balance, runout, vibration, thrust, motor current, temperature, and acoustic output. Begin with a commercially rated propeller matched to the motor. The generated parts have not undergone structural, flight, vibration, thrust, or acoustic validation.

## Build

GitHub Actions → **Build modular quadcopter redesign CAD**. The workflow runs on changes to this folder on demo and uploads the exports as the artifact **modular-quadcopter-redesign-cad**. Each part is exported in its own local coordinate frame for easier placement in a slicer.

The macro reports nominal XY bounding-box checks against the 200 × 200 mm bed. This does not replace checking each part's chosen print orientation, brim/supports, slicer placement, or actual printer clearance.
