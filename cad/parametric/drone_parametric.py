"""Parametric quadcopter model (rebuilt from the 'smooth 2' mesh measurements).

Axes: Y is up, X-Z is the horizontal plane (same as the source mesh). Units: mm.
Edit PARAMS (or pass --params file.json) and re-run to regenerate the model.
"""
import json, sys
import cadquery as cq

PARAMS = {
    # central body (plan polygon, mirrored left/right) ---------------------
    "body_height": 13.0,            # lower hull height (y 0..body_height)
    "body_half_width": 32.0,        # max half width (x) at the widest point
    "body_front_z": 66.0,           # nose (+z)
    "body_rear_z": -52.0,           # tail (-z)
    "body_front_half_width": 9.0,
    "body_rear_half_width": 19.0,
    "body_widest_z_front": 10.0,
    "body_widest_z_rear": -18.0,
    "body_edge_fillet": 0.8,
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
    # faceted (low-poly) dome, like the reference render --------------------------
    "canopy_faceted": True,
    "facet_vertices": 14,            # vertices per level (fewer = bigger flat facets)
    "facet_inset": 0.90,             # every 2nd vertex of the mid levels is pulled inward -> sharp creases
    "facet_ridge_lift": 1.6,         # extra height (mm) given to the top ridge vertices
    # rotor stations / arms ---------------------------------------------------
    "motor_x": 58.0,                # |x| of each motor axis
    "motor_z": 39.5,                # |z| of each motor axis
    "arm_width": 16.0,
    "arm_height": 12.0,
    "motor_radius": 6.0,
    "motor_height": 20.0,
    "motor_hub_radius": 9.0,        # base boss under the motor
    "motor_hub_height": 6.0,
    # propeller guards: TRUE circular rings -----------------------------------
    "guard_outer_radius": 35.5,     # used when guard_use_profile is False
    "guard_wall": 4.5,
    "guard_height": 17.0,
    "guard_edge_fillet": 1.8,
    # barrel profile measured on the original ring wall (radius vs height, mm)
    "guard_profile_y":     [0.0, 1.5, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0],
    "guard_profile_outer": [29.0, 32.1, 33.4, 34.4, 34.9, 35.3, 35.2, 34.2, 33.1, 31.0],
    "guard_profile_inner": [27.8, 27.8, 27.8, 28.0, 28.0, 27.5, 27.0, 27.0, 28.0, 28.5],
    "guard_use_profile": True,
    # front LED chevron (green light bar) ---------------------------------------
    "led_enabled": True,
    "led_points_xy": [[-9.0, 17.0], [-6.0, 14.8], [-3.0, 14.8], [0.0, 14.8], [3.0, 14.8], [6.0, 14.8], [9.0, 17.0]],  # angular \\___/ light bar
    "led_radius": 0.9,               # light-bar radius
    "led_groove_radius": 1.2,        # groove cut in the shell (insert sits in it)
    # fixed front camera, tilted down --------------------------------------------
    "camera_enabled": True,
    "camera_width": 22.0,
    "camera_height": 13.0,
    "camera_depth": 14.0,
    "camera_y": 7.0,
    "camera_z": 65.0,
    "camera_tilt_down_deg": 20.0,
    "camera_barrel_radius": 6.2,
    "camera_barrel_length": 5.0,
    "camera_lens_radius": 4.2,
    "camera_lens_protrusion": 1.2,
    # propellers (simple 2-blade, one per motor) --------------------------------
    "include_props": True,
    "prop_radius": 25.5,            # must stay < guard inner radius (guard_outer_radius - guard_wall)
    "prop_chord": 7.0,
    "prop_thickness": 1.2,
    "prop_y": 9.0,                 # blade plane height
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


def faceted_canopy(p):
    """Closed polyhedron of planar triangles through the measured levels -> crisp low-poly dome."""
    import os, numpy as np
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "body_sections.json")
    data = json.load(open(path))
    n = data["points_per_level"]; K = p["facet_vertices"]
    idx = [int(round(i * n / K)) % n for i in range(K)]
    def pick(pts):
        a = np.asarray(pts, float)[idx]
        b = a.copy()                       # mirror-symmetric about x=0 (pair vertex i with K-i)
        for i in range(K):
            k = (K - i) % K
            b[i, 0] = (a[i, 0] - a[k, 0]) / 2.0
            b[i, 1] = (a[i, 1] + a[k, 1]) / 2.0
        return b
    secs = sorted((float(y), np.array(q)) for y, q in data["sections"].items())
    lv = [(p["canopy_base_y"], pick(_resample(hull_polygon(p), n)))]
    for j, (y, q) in enumerate(secs):
        a = pick(q); c = a.mean(axis=0)
        a[1::2] = c + (a[1::2] - c) * p["facet_inset"]          # zig-zag -> creases between triangles
        lv.append((y, a))
    ytop, ptop = secs[-1]; a = pick(ptop); c = a.mean(axis=0)
    top = c + (a - c) * p["canopy_top_scale"]
    lv.append((p["canopy_top_y"], top))
    P = [[cq.Vector(float(x), float(y), float(z)) for (x, z) in pts] for y, pts in lv]
    # lift the two middle (ridge) vertices of the top ring a little
    faces = []
    def tri(a, b, c):
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon([a, b, c], close=True)))
    for L in range(len(P) - 1):
        for i in range(K):
            j = (i + 1) % K
            if i % 2 == 0:
                tri(P[L][i], P[L][j], P[L + 1][i]); tri(P[L][j], P[L + 1][j], P[L + 1][i])
            else:
                tri(P[L][i], P[L][j], P[L + 1][j]); tri(P[L][i], P[L + 1][j], P[L + 1][i])
    faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(P[0] + [P[0][0]], close=False) if False else cq.Wire.makePolygon(P[0], close=True)))
    faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(P[-1], close=True)))
    shell = cq.Shell.makeShell(faces)
    solid = cq.Solid.makeSolid(shell)
    if solid.Volume() < 0:
        solid = cq.Solid(solid.wrapped.Reversed())
    return cq.Workplane("XY").add(solid)


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
            if p.get("canopy_faceted"):
                return hull.union(faceted_canopy(p))
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


def guard_profile(p, x, z):
    """Round guard ring made by revolving the measured barrel profile -> exact circles."""
    ys, ro, ri = p["guard_profile_y"], p["guard_profile_outer"], p["guard_profile_inner"]
    outer = [(r, y) for r, y in zip(ro, ys)]
    inner = [(r, y) for r, y in zip(ri, ys)][::-1]
    w = (cq.Workplane("XY").moveTo(*outer[0]).spline(outer[1:], includeCurrent=True)
         .lineTo(*inner[0]).spline(inner[1:], includeCurrent=True).close())
    ring = w.revolve(360, (0, 0, 0), (0, 1, 0))
    return ring.translate((x, 0, z))


def guard(p, x, z):
    """Round guard ring: outer/inner radius are exact circles by construction."""
    if p.get("guard_use_profile"):
        return guard_profile(p, x, z)
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


def _surface_z(solid, x, y, zlo=30.0, zhi=95.0):
    """Front surface z of the model at (x, y): bisection on inside/outside."""
    lo, hi = zlo, zhi
    if not solid.isInside(cq.Vector(x, y, lo)):
        return None
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if solid.isInside(cq.Vector(x, y, mid)):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _led_path(p, solid):
    pts = []
    for x, y in p["led_points_xy"]:
        z = _surface_z(solid, x, y)
        if z is None:
            raise RuntimeError("LED point (%s,%s) is not on the shell" % (x, y))
        pts.append(cq.Vector(x, y, z))
    return pts


def _sweep_tube(pts, r):
    edges = [cq.Edge.makeLine(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    path = cq.Wire.assembleEdges(edges)
    d = (pts[1] - pts[0]).normalized()
    plane = cq.Plane(origin=pts[0], xDir=cq.Vector(0, 1, 0) if abs(d.y) < 0.9 else cq.Vector(1, 0, 0), normal=d)
    return cq.Workplane(plane).circle(r).sweep(cq.Workplane(obj=path), transition="round")


def led_parts(p, solid):
    """(groove tool, light-bar insert) following the front surface of the dome."""
    pts = _led_path(p, solid)
    return _sweep_tube(pts, p["led_groove_radius"]), _sweep_tube(pts, p["led_radius"])


def camera(p):
    w, h, dpt = p["camera_width"], p["camera_height"], p["camera_depth"]
    box = cq.Workplane("XY").box(w, h, dpt)
    box = box.edges("|Z").fillet(2.0)
    barrel = (cq.Workplane("XY").workplane(offset=dpt / 2).circle(p["camera_barrel_radius"]).extrude(p["camera_barrel_length"]))
    lens = (cq.Workplane("XY").workplane(offset=dpt / 2 + p["camera_barrel_length"]).circle(p["camera_lens_radius"]).extrude(p["camera_lens_protrusion"]))
    cam = box.union(barrel).union(lens)
    cam = cam.rotate((0, 0, 0), (1, 0, 0), p["camera_tilt_down_deg"])
    return cam.translate((0, p["camera_y"], p["camera_z"]))


def prop(p, x, z):
    blade = (cq.Workplane("XZ").workplane(offset=-p["prop_y"]).center(x, z)
             .rect(2 * p["prop_radius"], p["prop_chord"]).extrude(-p["prop_thickness"]))
    blade = blade.edges("|Y").fillet(min(p["prop_chord"], p["prop_thickness"] * 2) / 2.5)
    hub = (cq.Workplane("XZ").workplane(offset=-p["prop_y"] - p["prop_thickness"] / 2 + p["prop_hub_height"] / 2 - p["prop_hub_height"] / 2)
           .center(x, z).circle(p["prop_hub_radius"]).extrude(-p["prop_hub_height"]))
    return blade.union(hub)


def build(p=None):
    p = {**PARAMS, **(p or {})}
    inner_min = min(p["guard_profile_inner"]) if p.get("guard_use_profile") else p["guard_outer_radius"] - p["guard_wall"]
    assert p["prop_radius"] < inner_min, "prop would hit the guard"
    model = body(p)
    for i, (x, z) in enumerate(stations(p)):
        model = model.union(arm(p, x, z)).union(guard(p, x, z)).union(motor(p, x, z))
        if p["include_props"]:
            model = model.union(prop(p, x, z))
    if p["camera_enabled"]:
        model = model.union(camera(p))
    if p["led_enabled"]:
        groove, insert = led_parts(p, model.val())
        model = model.cut(groove)
        build.led_insert = insert              # exported separately (green part)
    return model


def export(shape, out_dir, name="drone_parametric"):
    import os
    os.makedirs(out_dir, exist_ok=True)
    cq.exporters.export(shape, os.path.join(out_dir, name + ".step"))
    cq.exporters.export(shape, os.path.join(out_dir, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
    try:                                    # close the tiny tessellation seams left by the swept LED groove
        import trimesh
        mesh = trimesh.load(os.path.join(out_dir, name + ".stl"))
        if not mesh.is_watertight:
            trimesh.repair.fill_holes(mesh)
            trimesh.repair.fix_normals(mesh)
            mesh.export(os.path.join(out_dir, name + ".stl"))
    except Exception as e:
        print("STL repair skipped:", e)
    led = getattr(build, "led_insert", None)
    if led is not None:
        cq.exporters.export(led, os.path.join(out_dir, "led_strip_green.step"))
        cq.exporters.export(led, os.path.join(out_dir, "led_strip_green.stl"), tolerance=0.02, angularTolerance=0.1)


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

# Harmless commit to trigger the demo-branch regeneration workflow; no model changes.

# Workflow trigger: rerun smooth2 parametric CAD exports after cache configuration fix.

# Trigger regeneration after making the CAD export push resilient to concurrent branch updates.
