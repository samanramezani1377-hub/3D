"""Sanity tests for the Smooth2 propeller design (run: python -m pytest cad/parametric or python this file)."""
import json
import math
from pathlib import Path

import prop_bemt as pb

PARAMS = json.loads(Path(__file__).with_name("model.params.json").read_text(encoding="utf-8"))
SCALE = PARAMS["model_scale"]
RADIUS = PARAMS["prop_radius"] * SCALE / 1000.0
HUB = PARAMS["prop_hub_radius"] * SCALE / 1000.0
ROWS = PARAMS["prop_blade_stations"]


def test_bemt_matches_report_for_original_blade():
    """The original constant-pitch blade must give ~0.20 N at 9979 RPM, as the report states."""
    old = [(0.05, 6.96), (0.14, 8.36), (0.23, 9.21), (0.33, 9.43), (0.44, 8.79), (0.56, 7.82),
           (0.68, 6.86), (0.80, 6.0), (0.90, 5.14), (0.96, 4.71), (1.0, 4.28)]
    stations = [(x, c * SCALE / 1000.0) for x, c in old]
    betas = pb.constant_pitch_betas(RADIUS, stations, 0.053086, cap_deg=42)
    result = pb.analyse(RADIUS, stations, betas, 9979, hub_radius=0.0)
    assert abs(result["thrust_N"] - 0.200) < 0.005


def test_new_stations_are_designed_not_capped():
    betas = [r[2] for r in ROWS]
    assert all(len(r) == 3 for r in ROWS)
    assert all(b2 <= b1 for b1, b2 in zip(betas, betas[1:])), "pitch must decrease outward"
    assert max(betas) < 42.0 and min(betas) > 5.0
    assert ROWS[0][0] < PARAMS["prop_hub_radius"] / PARAMS["prop_radius"], "root starts inside the hub"


def test_no_station_is_stalled_at_scenario_rpm():
    stations = [(r[0], r[1] * SCALE / 1000.0) for r in ROWS]
    alpha0 = pb.thin_airfoil_alpha0(PARAMS["prop_airfoil_camber"], PARAMS["prop_airfoil_camber_position"])
    result = pb.analyse(RADIUS, stations, [r[2] for r in ROWS], 8403, hub_radius=HUB,
                        alpha0=alpha0, prandtl=True)
    assert all(abs(cl) < 1.19 for _, _, cl in result["detail"])
    assert result["thrust_N"] > 0.1


def test_zero_lift_angle_is_negative_for_positive_camber():
    assert pb.thin_airfoil_alpha0(0.04, 0.4) < math.radians(-3.0)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
