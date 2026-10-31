"""Derive the Smooth2 blade stations (r/R, chord, pitch angle) from the BEMT design tool.

Run from the repository root:  python cad/parametric/design_prop.py

It reads ``prop_design`` from model.params.json, rewrites ``prop_blade_stations`` as
rows ``[r/R, chord_local, beta_deg]`` and keeps every other parameter unchanged.  The CAD
generator then builds exactly those stations; nothing is hard-coded in the generator.

Design rules (all screening-level, not bench-validated):
  * blade root starts inside the hub and the twist is NOT capped at an arbitrary angle;
  * every station is pitched so it works at one design lift coefficient at the design RPM;
  * chord is scaled by ``chord_scale`` from the original planform (diameter unchanged).
"""
import json
import math
from pathlib import Path

import prop_bemt as pb

PARAMS_PATH = Path(__file__).resolve().with_name("model.params.json")
# Planform kept from the first design (r/R, chord in local model units).
BASE_PLANFORM = [(0.14, 8.36), (0.23, 9.21), (0.33, 9.43), (0.44, 8.79), (0.56, 7.82),
                 (0.68, 6.86), (0.80, 6.00), (0.90, 5.14), (0.96, 4.71), (1.00, 4.28)]


def _interp(rows, x):
    if x <= rows[0][0]:
        return rows[0][1]
    for (x0, y0), (x1, y1) in zip(rows, rows[1:]):
        if x <= x1:
            t = (x - x0) / (x1 - x0)
            return y0 * (1 - t) + y1 * t
    return rows[-1][1]


def design(params):
    dp = params["prop_design"]
    scale = float(params["model_scale"])
    radius_local = float(params["prop_radius"])
    radius_m = radius_local * scale / 1000.0
    hub_frac = float(params["prop_hub_radius"]) / radius_local
    root_x = round(hub_frac * float(dp["root_start_fraction_of_hub"]), 4)
    xs = [root_x] + [float(v) for v in dp["radial_fractions"]]
    chords = [_interp(BASE_PLANFORM, x) * float(dp["chord_scale"]) for x in xs]
    stations_m = [(x, c * scale / 1000.0) for x, c in zip(xs, chords)]
    alpha0 = pb.thin_airfoil_alpha0(float(params["prop_airfoil_camber"]),
                                    float(params["prop_airfoil_camber_position"]))
    betas = pb.design_twist(radius_m, stations_m, float(dp["design_rpm"]),
                            float(dp["design_cl"]), alpha0, prandtl=False)
    rows = [[round(x, 4), round(c, 3), round(b, 2)] for x, c, b in zip(xs, chords, betas)]
    # Effective geometric pitch at 0.75R, final millimetres (documentation value).
    b75 = _interp([(r[0], r[2]) for r in rows], 0.75)
    pitch_mm = 2.0 * math.pi * 0.75 * radius_m * 1000.0 * math.tan(math.radians(b75))
    return rows, pitch_mm


def main():
    params = json.loads(PARAMS_PATH.read_text(encoding="utf-8"))
    rows, pitch_mm = design(params)
    params["prop_blade_stations"] = rows
    params["prop_pitch_mm"] = round(pitch_mm, 3)
    PARAMS_PATH.write_text(json.dumps(params, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for row in rows:
        print(row)
    print("pitch@0.75R mm:", round(pitch_mm, 3))


if __name__ == "__main__":
    main()
