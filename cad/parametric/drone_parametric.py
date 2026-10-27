"""Parametric quadcopter model (rebuilt from the 'smooth 2' mesh measurements).

Axes: Y is up, X-Z is the horizontal plane (same as the source mesh). Units: mm.
Edit PARAMS (or pass --params file.json) and re-run to regenerate the model.
"""
import json, sys
import cadquery as cq

PARAMS = {
    # central body (plan polygon, mirrored left/right) ---------------------
    "body_height": 13.0,            # lower hull height (y 0..body_height)
    "body_half_width": 22.0,        # max half width (x) at the widest point
    "body_front_z": 62.0,           # nose (+z)
    "body_rear_z": -45.0,           # tail (-z)
    "body_front_half_width": 12.0,
    "body_rear_half_width": 18.0,
    "body_widest_z_front": 20.0,
    "body_widest_z_rear": -10.0,
    "body_edge_fillet": 3.0,
    # canopy (upper dome) ---------------------------------------------------
    "canopy_height": 12.0,
    "canopy_front_z": 44.0,
    "canopy_rear_z": -20.0,
    "canopy_half_width": 19.0,
    "canopy_taper_deg": 18.0,
    "canopy_fillet": 4.0,
    # rotor stations / arms ---------------------------------------------------
    "motor_x": 37.0,                # |x| of each motor axis
    "motor_z": 39.5,                # |z| of each motor axis
    "arm_width": 12.0,
    "arm_height": 9.0,
    "motor_radius": 6.0,
    "motor_height": 20.0,
    "motor_hub_radius": 9.0,        # base boss under the motor
    "motor_hub_height": 6.0,
    # propeller guards: TRUE circular rings -----------------------------------
    "guard_outer_radius": 35.5,
    "guard_wall": 4.5,
    "guard_height": 17.0,
    "guard_edge_fillet": 1.8,
}


def body(p):
    pts = [(-p["body_front_half_width"], p["body_front_z"]),
           (p["body_front_half_width"], p["body_front_z"]),
           (p["body_half_width"], p["body_widest_z_front"]),
           (p["body_half_width"], p["body_widest_z_rear"]),
           (p["body_rear_half_width"], p["body_rear_z"]),
           (-p["body_rear_half_width"], p["body_rear_z"]),
           (-p["body_half_width"], p["body_widest_z_rear"]),
           (-p["body_half_width"], p["body_widest_z_front"])]
    # workplane XZ has its normal along -Y, so extrude negative to go +Y
    hull = (cq.Workplane("XZ").polyline(pts).close().extrude(-p["body_height"]))
    try:
        hull = hull.edges("|Y").fillet(p["body_edge_fillet"])
    except Exception:
        pass
    cpts = [(-p["canopy_half_width"] * 0.6, p["canopy_front_z"]),
            (p["canopy_half_width"] * 0.6, p["canopy_front_z"]),
            (p["canopy_half_width"], 0.0),
            (p["canopy_half_width"] * 0.9, p["canopy_rear_z"]),
            (-p["canopy_half_width"] * 0.9, p["canopy_rear_z"]),
            (-p["canopy_half_width"], 0.0)]
    canopy = (cq.Workplane("XZ").workplane(offset=-p["body_height"]).polyline(cpts).close()
              .extrude(-p["canopy_height"], taper=p["canopy_taper_deg"]))
    try:
        canopy = canopy.faces(">Y").edges().fillet(p["canopy_fillet"])
    except Exception:
        pass
    return hull.union(canopy)


def stations(p):
    return [(sx * p["motor_x"], sz * p["motor_z"]) for sx in (1, -1) for sz in (1, -1)]


def guard(p, x, z):
    """Round guard ring: outer/inner radius are exact circles by construction."""
    ro, ri = p["guard_outer_radius"], p["guard_outer_radius"] - p["guard_wall"]
    ring = (cq.Workplane("XZ").center(x, z).circle(ro).circle(ri).extrude(-p["guard_height"]))
    try:
        ring = ring.faces(">Y").edges().fillet(p["guard_edge_fillet"])
    except Exception:
        pass
    return ring


def arm(p, x, z):
    """Straight bar from the body centre line to the motor axis."""
    import math
    L = math.hypot(x, z)
    ang = math.degrees(math.atan2(z, x))
    bar = (cq.Workplane("XY").box(L, p["arm_width"], p["arm_height"], centered=(False, True, False))
           .rotate((0, 0, 0), (0, 0, 1), 0))
    # box is built along +X in the XY plane with Z up; map to our Y-up frame:
    # build in (x, z) plan with height along Y by rotating about X.
    bar = bar.rotate((0, 0, 0), (1, 0, 0), -90)          # box height Z -> Y, width Y -> -Z
    bar = bar.rotate((0, 0, 0), (0, 1, 0), -ang)          # aim at the motor (x, z)
    return bar


def motor(p, x, z):
    hub = (cq.Workplane("XZ").center(x, z).circle(p["motor_hub_radius"]).extrude(-p["motor_hub_height"]))
    can = (cq.Workplane("XZ").center(x, z).circle(p["motor_radius"]).extrude(-p["motor_height"]))
    return hub.union(can)


def build(p=None):
    p = {**PARAMS, **(p or {})}
    model = body(p)
    for (x, z) in stations(p):
        model = model.union(arm(p, x, z)).union(guard(p, x, z)).union(motor(p, x, z))
    return model


if __name__ == "__main__":
    params = {}
    if len(sys.argv) > 2 and sys.argv[1] == "--params":
        params = json.load(open(sys.argv[2]))
    shape = build(params)
    bb = shape.val().BoundingBox()
    print("bbox", bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax, "valid", shape.val().isValid())
