# Quiet propellers for small multirotors — research notes (checked 2026-10-27)

## Evidence worth considering
- Candeloro, Ragni & Pagliaroli, *Experimental investigation on the application of serrated trailing edge propellers for drone noise reduction*, Applied Acoustics, published 15 Jan 2026. Their experiments report about 3–4 dB reductions in overall sound pressure for tuned serrations, but also report meaningful thrust penalties in some forward-flight conditions. DOI: https://doi.org/10.1016/j.apacoust.2025.111045
- Shekh & Rani, *Optimization of UAV propeller performance through design, development, and testing of diverse propeller geometries*, first published 18 Dec 2025. The paper compares plain, tilted-tip, serrated and toroidal designs; its tested toroidal configuration reports about 2–3 dB reduction, but the result is specific to their geometry and test conditions. DOI: https://doi.org/10.1177/09544100251400497
- *A comprehensive design method to balance efficiency and noise for high-thrust electric UAV propellers*, Energy, 2025. Reports a jointly optimized geometry with 4.8 dB reduction under its test conditions. DOI: https://doi.org/10.1016/j.energy.2025.136444
- Brown & Withey, Quiet Drones 2026, *Design and optimisation of a low noise unmanned aerial system propeller*. A simulated 3-inch propeller with a tip elbow showed up to 5.66 dB reduction versus its baseline; this is not a guarantee for a different rotor/duct.
- Delft/TU Delft researchers' psychoacoustic evaluation of a low-noise drone propeller reports lower perceived-noise metrics in measured directions while maintaining the target thrust: https://research.tudelft.nl/files/229249642/full_paper_365_20240331054028380.pdf

## Practical recommendation for this design
1. Do not assume a toroidal or serrated blade is automatically quieter. Noise depends on rotor diameter, RPM, blade loading, motor/ESC tone, guard interaction, frame vibration and microphone direction.
2. Start with a commercially manufactured low-noise propeller matched to the motor and required thrust. Larger diameter at lower RPM can often help if the frame and motors allow it.
3. Keep the guard smooth and stiff, and preserve generous, even tip clearance; a poorly placed duct/guard can increase interaction noise and reduce thrust.
4. Avoid aggressive serrations on a first printable prototype. They need geometry-specific testing and may sacrifice thrust. Any experimental blade should be designed and tested by an experienced aero team in a guarded thrust stand.
5. Compare candidate props at equal thrust, not equal throttle/RPM alone. Record A-weighted SPL and narrowband spectra at fixed distance and direction, along with thrust, RPM, current and voltage. Repeat runs and document the setup.
6. Printed propellers are not recommended for flight tests: layer interfaces, imbalance and defects can cause blade release at high RPM. Use certified/commercial rotors for powered tests.

## Conclusion
For the first demo build, use a standard balanced propeller and focus on reducing RPM for the same thrust, rigid mounting, motor vibration, and guard clearance. Treat serrated/toroidal/tip-elbow designs as research candidates to test, not proven drop-in upgrades.
