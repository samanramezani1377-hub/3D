"""Parametric quadcopter model (rebuilt from the 'smooth 2' mesh measurements).

Axes: Y is up, X-Z is the horizontal plane (same as the source mesh). Units: mm.
Edit PARAMS (or pass --params file.json) and re-run to regenerate the model.
"""
import json, sys
import cadquery as cq

PARAMS = {
    # central body (plan polygon, mirrored left/right) ---------------------
    "body_height": 13.0,            # lower hull height (y 0..body_height)
    "body_half_width": 24.0,        # max half width (x) at the widest point
    "body_front_z": 66.0,           # nose (+z)
    "body_rear_z": -52.0,           # tail (-z)
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
    "use_measured_canopy": True,     # loft through cad/parametric/body_sections.json (edit the numbers to reshape)
    "canopy_base_y": 13.0,
    "canopy_top_y": 25.6,
    "canopy_top_scale": 0.45,        # last ring of the dome relative to the top measured section
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
    # propellers (simple 2-blade, one per motor) --------------------------------
    "include_props": True,
    "prop_radius": 29.0,            # must stay < guard inner radius (guard_outer_radius - guard_wall)
    "prop_chord": 7.0,
    "prop_thickness": 1.2,
    "prop_y": 14.0,                 # blade plane height
    "prop_hub_radius": 4.0,
    "prop_hub_height": 4.0,
}


def _resample(pts, n):
    import numpy as np
    pts = np.asarray(pts, float)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    if area < 0:
        pts = pts[::-1]
    pts = np.roll(pts, -int(np.argmax(pts[:, 1])), axis=0)
    line = np.vstack([pts, pts[:1]])
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(line, axis=0).T))]
    t = np.linspace(0, seg[-1], n, endpoint=False)
    return np.c_[np.interp(t, seg, line[:, 0]), np.interp(t, seg, line[:, 1])]


def hull_polygon(p):
    return [(-p["body_front_half_width"], p["body_front_z"]),
            (p["body_front_half_width"], p["body_front_z"]),
            (p["body_half_width"], p["body_widest_z_front"]),
            (p["body_half_width"], p["body_widest_z_rear"]),
            (p["body_rear_half_width"], p["body_rear_z"]),
            (-p["body_rear_half_width"], p["body_rear_z"]),
            (-p["body_half_width"], p["body_widest_z_rear"]),
            (-p["body_half_width"], p["body_widest_z_front"])]


def measured_canopy(p):
    import os, numpy as np
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "body_sections.json")
    data = json.load(open(path))
    n = data["points_per_level"]
    levels = [(p["canopy_base_y"], _resample(hull_polygon(p), n))]
    secs = [(float(y), np.array(pts)) for y, pts in data["sections"].items()]
    secs.sort()
    levels += secs
    ytop, ptop = secs[-1]
    c = ptop.mean(axis=0)
    levels.append((p["canopy_top_y"], c + (ptop - c) * p["canopy_top_scale"]))
    wp = None
    for i, (y, pts) in enumerate(levels):
        if wp is None:
            wp = cq.Workplane("XZ").workplane(offset=-y)
        else:
            wp = wp.workplane(offset=-(y - levels[i - 1][0]))
        wp = wp.polyline([tuple(q) for q in pts]).close()
    return wp.loft(ruled=False, combine=True)


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
    if p.get("use_measured_canopy"):
        try:
            return hull.union(measured_canopy(p))
        except Exception as e:  # fall back to the simple tapered canopy below
            print("measured canopy failed, using simple canopy:", e)
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


def prop(p, x, z):
    blade = (cq.Workplane("XZ").workplane(offset=-p["prop_y"]).center(x, z)
             .rect(2 * p["prop_radius"], p["prop_chord"]).extrude(-p["prop_thickness"]))
    blade = blade.edges("|Y").fillet(min(p["prop_chord"], p["prop_thickness"] * 2) / 2.5)
    hub = (cq.Workplane("XZ").workplane(offset=-p["prop_y"] - p["prop_thickness"] / 2 + p["prop_hub_height"] / 2 - p["prop_hub_height"] / 2)
           .center(x, z).circle(p["prop_hub_radius"]).extrude(-p["prop_hub_height"]))
    return blade.union(hub)


def build(p=None):
    p = {**PARAMS, **(p or {})}
    assert p["prop_radius"] < p["guard_outer_radius"] - p["guard_wall"], "prop would hit the guard"
    model = body(p)
    for i, (x, z) in enumerate(stations(p)):
        model = model.union(arm(p, x, z)).union(guard(p, x, z)).union(motor(p, x, z))
        if p["include_props"]:
            model = model.union(prop(p, x, z))
    return model


def export(shape, out_dir, name="drone_parametric"):
    import os
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(shape, os.path.join(out_dir, name + ".step"))
    cq.exporters.export(shape, os.path.join(out_dir, name + ".stl"), tolerance=0.05, angularTolerance=0.1)


if __name__ == "__main__":
    params = {}
    if len(sys.argv) > 2 and sys.argv[1] == "--params":
        params = json.load(open(sys.argv[2]))
    shape = build(params)
    bb = shape.val().BoundingBox()
    print("bbox", bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax, "valid", shape.val().isValid())
    out = "output/قالب گیری شده/smooth2-parametric"
    export(shape, out)
    json.dump({**PARAMS, **params}, open(out + "/drone_parametric.params.json", "w"), indent=2)
