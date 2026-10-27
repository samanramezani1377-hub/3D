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


def build(p=None):
    p = {**PARAMS, **(p or {})}
    return body(p)


if __name__ == "__main__":
    params = {}
    if len(sys.argv) > 2 and sys.argv[1] == "--params":
        params = json.load(open(sys.argv[2]))
    shape = build(params)
    bb = shape.val().BoundingBox()
    print("bbox", bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax, "valid", shape.val().isValid())
