"""Parametric quadcopter model (rebuilt from the 'smooth 2' mesh measurements).

Design coordinates are Y-up with X-Z as the plan; exported CAD is rotated to Z-up for
common CAD/3D-printing orientation. Export transform: (x, y, z) -> (x, -z, y).
Thus the camera/front points toward -Y and the flat underside faces -Z. Units: mm.
Edit PARAMS (or pass --params file.json) and re-run to regenerate the model.
"""
import json, sys
import cadquery as cq

PARAMS = {
    # central body (plan polygon, mirrored left/right) ---------------------
    "body_height": 13.0,            # lower hull height (y 0..body_height)
    "body_shell_thickness": 2.0,    # nominal thickness of shell, flat bottom plate, and roof
    "body_half_width": 38.0,        # max half width (x) at the widest point (was 32: middle made wider)
    "body_front_z": 66.0,           # nose (+z)
    "body_rear_z": -52.0,           # tail (-z)
    "body_front_half_width": 9.0,
    "body_rear_half_width": 16.0,
    "body_widest_z_front": 10.0,
    "body_widest_z_rear": -18.0,
    "body_edge_fillet": 0.8,
    # extra widening of the dome through the middle of the body (plan view: narrow nose/tail, fat middle)
    "mid_widen": 0.22,              # relative extra half width at the middle (0 = off)
    "mid_widen_z": -4.0,            # z of the widest point of the dome
    "mid_widen_sigma": 30.0,        # how far (mm) the widening spreads front/back
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
    "motor_z": 58.0,                # |z| of each motor axis (= motor_x -> square 4-rotor layout, was 39.5)
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
    # enlarge the guards in place (radii of the measured profile shifted outward, wall thickness kept) until they
    # almost touch the body (0.6 mm gap, not fused): +2.7 model units -> outer radius ~48.9 mm final
    "guard_grow": 2.7,
    # front LED slit (no separate part): a small slit through the front shell that opens into a small
    # pocket behind it, so a real LED can be glued from inside later. Sizes are in FINAL (scaled) mm.
    "led_enabled": True,
    "led_y": 14.5,                   # height of the slit centre (model units, before scaling)
    "led_slit_length": 10.0,         # slit width along x
    "led_slit_height": 2.0,          # slit height
    "led_wall": 1.5,                 # shell thickness the slit passes through
    "led_pocket_length": 12.0,       # pocket behind the slit (x)
    "led_pocket_height": 4.0,        # (y)
    "led_pocket_depth": 6.0,         # how far the pocket goes inward (z)
    # front camera: small module hidden INSIDE the nose, only a small square housing (aligned with the lens axis) shows outside.
    # Sizes are in FINAL (scaled) mm. The lens axis points forward, tilted down by camera_tilt_down_deg.
    "camera_enabled": True,
    "camera_tilt_down_deg": 35.0,
    "camera_y": 8.35,                # center the lens housing vertically on the scaled nose face (body_height * model_scale / 2)
    "camera_block_size": 10.5,       # side of the square housing that sticks out (aligned with the lens axis)
    "camera_block_protrusion": 3.5,  # how far the housing sticks out of the nose face
    "camera_block_fillet": 0.8,      # rounding of the housing's corners
    "camera_lens_hole_radius": 3.2,  # round lens opening inside the housing
    "camera_wall": 1.5,              # shell thickness between the outside and the camera pocket
    "camera_cavity_w": 9.5,          # internal pocket for the camera module (width / height / depth)
    "camera_cavity_h": 6.0,
    "camera_cavity_d": 7.0,
    # propellers (simple 2-blade, one per motor) --------------------------------
    "include_props": True,
    "prop_radius": 25.5,            # must stay < guard inner radius (guard_outer_radius - guard_wall)
    "prop_chord": 7.0,
    "prop_thickness": 1.2,
    "prop_y": 9.0,                 # blade plane height
    "prop_hub_radius": 4.0,
    "prop_hub_height": 4.0,
    # overall size -------------------------------------------------------------
    # model_scale: fixed uniform scale to final mm. It keeps the body at its approved size (it was the scale that
    # made the old layout exactly 240 mm). The bigger guards now make the footprint ~247 x 247 mm (limit lifted).
    # target_overall_mm: if set (and model_scale is None) the scale is derived so the largest extent equals it.
    # Floor-mounted arm attachment: integral tongue + two vertical M3 fasteners per arm.
    # Dimensions below are pre-scale model units; model_scale converts them to final millimetres.
    "joint_clearance": 0.25,
    "joint_tip_width": 10.0,
    "joint_lead_in_length": 6.0,
    "joint_tab_start_radius": 20.5,
    "joint_tab_end_radius": 43.0,
    "joint_tab_width": 16.5,
    "joint_tab_thickness": 2.5,
    "joint_tab_y": 4.0,
    "joint_floor_pad_height": 2.0,
    "joint_floor_pad_width": 16.5,
    "joint_floor_pad_end_radius": 36.5,
    "joint_arm_start_radius": 37.0,
    "joint_entry_start_radius": 36.5,
    "joint_entry_end_radius": 50.0,

    "joint_bolt_radii": [25.5, 32.5],
    "joint_bolt_hole_radius": 1.3,
    "joint_nut_af": 4.6,
    "joint_nut_depth": 2.0,
    "joint_nut_hole_radius": 1.2,
    "joint_screw_shaft_radius": 1.15,
    "joint_screw_length": 3.9,
    "joint_screw_head_radius": 2.15,
    "joint_screw_head_height": 2.3,
    "joint_screw_socket_af": 1.95,
    "joint_screw_socket_depth": 0.9,
    "joint_nut_thickness": 1.9,
    "model_scale": 1.28548,
    "target_overall_mm": None,
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


def widen_x(p, pts):
    """Scale x of dome section points by a smooth bump centred on the body middle."""
    import numpy as np
    a = np.array(pts, float)
    k, zc, sg = p.get("mid_widen", 0.0), p["mid_widen_z"], p["mid_widen_sigma"]
    a[:, 0] *= 1.0 + k * np.exp(-((a[:, 1] - zc) / sg) ** 2)
    return a


def hull_polygon(p):
    return [(-p["body_front_half_width"], p["body_front_z"]),
            (p["body_front_half_width"], p["body_front_z"]),
            (p["body_half_width"], p["body_widest_z_front"]),
            (p["body_half_width"], p["body_widest_z_rear"]),
            (p["body_rear_half_width"], p["body_rear_z"]),
            (-p["body_rear_half_width"], p["body_rear_z"]),
            (-p["body_half_width"], p["body_widest_z_rear"]),
            (-p["body_half_width"], p["body_widest_z_front"])]


def canopy_base_points(p):
    """Shared 14-vertex footprint used by both the lower hull and canopy rim."""
    import os, numpy as np
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "body_sections.json")
    data = json.load(open(path))
    n = data["points_per_level"]
    K = p["facet_vertices"]
    idx = [int(round(i * n / K)) % n for i in range(K)]
    a = _resample(hull_polygon(p), n)[idx]
    # Keep the exact same mirrored point treatment as the faceted canopy rings.
    b = a.copy()
    for i in range(K):
        k = (K - i) % K
        b[i, 0] = (a[i, 0] - a[k, 0]) / 2.0
        b[i, 1] = (a[i, 1] + a[k, 1]) / 2.0
    return [tuple(map(float, q)) for q in b]


def faceted_canopy(p, inward=0.0, open_bottom_y=None, roof_drop=0.0):
    """Closed polyhedron through measured levels; inward builds an inset cavity for the hollow body."""
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
    base_y = p["canopy_base_y"] if open_bottom_y is None else open_bottom_y
    def inset_ring(a):
        if inward <= 0:
            return a
        c = a.mean(axis=0)
        radii = np.linalg.norm(a - c, axis=1)
        return c + (a - c) * np.maximum(0.1, (radii - inward) / np.maximum(radii, 1e-6))[:, None]
    base_pts = np.asarray(canopy_base_points(p), float)
    lv = [(base_y, inset_ring(base_pts))]
    for j, (y, q) in enumerate(secs):
        a = widen_x(p, pick(q)); c = a.mean(axis=0)
        a[1::2] = c + (a[1::2] - c) * p["facet_inset"]
        lv.append((y, inset_ring(a)))
    ytop, ptop = secs[-1]; a = widen_x(p, pick(ptop)); c = a.mean(axis=0)
    top = inset_ring(c + (a - c) * p["canopy_top_scale"])
    lv.append((p["canopy_top_y"] - roof_drop, top))
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
    secs = [(float(y), widen_x(p, pts)) for y, pts in data["sections"].items()]
    secs.sort(key=lambda t: t[0])
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
    # Match the lower hull's top rim to the exact canopy base polygon.
    pts = canopy_base_points(p)
    # Preserve the approved exterior geometry.
    hull = cq.Workplane("XZ").polyline(pts).close().extrude(-p["body_height"])
    if p.get("use_measured_canopy"):
        try:
            canopy = faceted_canopy(p) if p.get("canopy_faceted") else measured_canopy(p)
            outer = hull.union(canopy)
        except Exception as e:
            print("measured canopy failed, using the simple canopy fallback:", e)
            outer = hull
    else:
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
        outer = hull.union(canopy)

    # Hollow the body internally while retaining a continuous, flat bottom plate.
    # The same nominal wall thickness is kept on the floor, side walls, and roof.
    wall = float(p.get("body_shell_thickness", 2.0))
    if wall <= 0 or wall >= min(p["body_half_width"], p["body_height"]):
        raise ValueError("body_shell_thickness must be positive and smaller than the body")
    cz = (p["body_front_z"] + p["body_rear_z"]) / 2.0
    scale_x = max(0.1, 1.0 - wall / p["body_half_width"])
    scale_z = max(0.1, 1.0 - wall / ((p["body_front_z"] - p["body_rear_z"]) / 2.0))
    inner_pts = [(x * scale_x, cz + (z - cz) * scale_z) for x, z in pts]
    # Workplane XZ extrudes along -Y; negative extrusion runs upward.
    # Start the cavity at y=wall (not below y=0) so the outside underside stays closed and planar.
    lower_void = cq.Workplane("XZ").polyline(inner_pts).close().extrude(-(p["body_height"] + 2.0))
    lower_void = lower_void.translate((0, wall, 0))
    if p.get("use_measured_canopy") and p.get("canopy_faceted"):
        roof_void = faceted_canopy(p, inward=wall, open_bottom_y=p["body_height"] - 1.0,
                                   roof_drop=wall)
        cavity = lower_void.union(roof_void)
        hollow = outer.cut(cavity)
    else:
        # Keep the fallback canopy stable; at minimum the lower body is hollow.
        hollow = outer.cut(lower_void)
    if not hollow.val().isValid():
        raise RuntimeError("Hollow body geometry is invalid")
    return hollow


def stations(p):
    return [(sx * p["motor_x"], sz * p["motor_z"]) for sx in (1, -1) for sz in (1, -1)]


def guard_profile(p, x, z):
    """Round guard ring made by revolving the measured barrel profile -> exact circles."""
    g = p.get("guard_grow", 0.0)
    ys = p["guard_profile_y"]
    ro = [r + g for r in p["guard_profile_outer"]]
    ri = [r + g for r in p["guard_profile_inner"]]
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


# The arm and body socket share this rounded, tapered plan-view profile.
def radial_joint_profile(length, width, tip_width, lead_length):
    """Plan-view outline with a rounded inner nose and tapered lead-in; extrude along local Z."""
    if length <= 0 or width <= 0 or tip_width <= 0 or tip_width >= width:
        raise ValueError("invalid radial joint profile dimensions")
    lead = min(max(lead_length, tip_width / 2), length)
    r = tip_width / 2
    return (cq.Workplane("XY")
            .moveTo(r, -r)
            .threePointArc((0, 0), (r, r))
            .lineTo(lead, width / 2)
            .lineTo(length, width / 2)
            .lineTo(length, -width / 2)
            .lineTo(lead, -width / 2)
            .close())


def arm(p, x, z, start_r=0.0, y_offset=0.0):
    """Main arm beam. Modular arms enter through the side opening at inner-floor level."""
    import math
    L = math.hypot(x, z)
    ang = math.degrees(math.atan2(z, x))
    if start_r < 0 or start_r >= L:
        raise ValueError("arm start radius must be within the motor station")
    tip_width = p.get("joint_tip_width", 10.0)
    lead_length = p.get("joint_lead_in_length", 6.0)
    bar_height = p["arm_height"] if y_offset == 0.0 else p["arm_height"] - y_offset - 1.0
    if bar_height <= 0:
        raise ValueError("arm root height must remain positive")
    bar = radial_joint_profile(L - start_r, p["arm_width"], tip_width, lead_length).extrude(bar_height)
    bar = bar.rotate((0, 0, 0), (1, 0, 0), -90)
    bar = bar.rotate((0, 0, 0), (0, 1, 0), -ang)
    bar = bar.translate((start_r * math.cos(math.radians(ang)), y_offset,
                         start_r * math.sin(math.radians(ang))))
    return bar


def radial_part(profile, x, z, angle_deg, y_base):
    """Place a radial/tangential profile in the model's Y-up coordinate frame."""
    return (profile.rotate((0, 0, 0), (1, 0, 0), -90)
            .rotate((0, 0, 0), (0, 1, 0), -angle_deg)
            .translate((x, y_base, z)))


def mounting_tab(p, x, z):
    """Integral flat arm tongue with two vertical clearance holes."""
    import math
    angle = math.degrees(math.atan2(z, x))
    start, end = p["joint_tab_start_radius"], p["joint_tab_end_radius"]
    tab = radial_joint_profile(end - start, p["joint_tab_width"],
                               p["joint_tip_width"], p["joint_lead_in_length"]).extrude(p["joint_tab_thickness"])
    tab = radial_part(tab, start * math.cos(math.radians(angle)),
                      start * math.sin(math.radians(angle)), angle, p["joint_tab_y"])
    for radius in p["joint_bolt_radii"]:
        hx, hz = radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle))
        hole = (cq.Workplane("XZ").workplane(offset=-(p["joint_tab_y"] - 0.05))
                .center(hx, hz).circle(p["joint_bolt_hole_radius"])
                .extrude(-(p["joint_tab_thickness"] + 0.1)))
        tab = tab.cut(hole)
    return tab


def floor_mounting_rail(p, x, z):
    """Raised floor rib under the tongue; the body later receives matching hex-nut pockets."""
    import math
    angle = math.degrees(math.atan2(z, x))
    start, end = p["joint_tab_start_radius"], p["joint_floor_pad_end_radius"]
    rail = radial_joint_profile(end - start, p["joint_floor_pad_width"],
                                p["joint_tip_width"], p["joint_lead_in_length"]).extrude(
                                    p["joint_floor_pad_height"] + 0.02)
    return radial_part(rail, start * math.cos(math.radians(angle)),
                       start * math.sin(math.radians(angle)), angle,
                       p["body_shell_thickness"] - 0.02)


def arm_entry_slot(p, x, z):
    """Side-wall opening for the arm beam; the floor plate remains intact."""
    import math
    angle = math.degrees(math.atan2(z, x))
    start, end = p["joint_entry_start_radius"], p["joint_entry_end_radius"]
    floor_y = p["body_shell_thickness"]
    top_y = p["body_height"] + 0.2
    cutter = cq.Workplane("XY").box(end - start, top_y - floor_y, p["arm_width"] + 2 * p["joint_clearance"])
    cutter = cutter.rotate((0, 0, 0), (0, 1, 0), -angle)
    return cutter.translate(((start + end) / 2 * math.cos(math.radians(angle)),
                             (floor_y + top_y) / 2,
                             (start + end) / 2 * math.sin(math.radians(angle))))


def nut_part(p, x, z):
    """Simplified nut envelope for assembly visualization; use real metal M3 nuts in the printed frame."""
    import math
    circ_dia = p["joint_nut_af"] / math.cos(math.radians(30.0))
    thickness = p["joint_nut_thickness"]
    bottom = p["joint_tab_y"] - p["joint_nut_depth"] + (p["joint_nut_depth"] - thickness) / 2
    nut = (cq.Workplane("XZ").workplane(offset=-bottom).center(x, z)
           .polygon(6, circ_dia).extrude(-thickness))
    bore = (cq.Workplane("XZ").workplane(offset=-(bottom - 0.01)).center(x, z)
            .circle(p["joint_nut_hole_radius"]).extrude(-(thickness + 0.02)))
    return nut.cut(bore)


def screw_part(p, x, z):
    """Simplified vertical socket-head screw envelope, accessible from the top."""
    import math
    shaft_r, head_r = p["joint_screw_shaft_radius"], p["joint_screw_head_radius"]
    head_h = p["joint_screw_head_height"]
    tab_top = p["joint_tab_y"] + p["joint_tab_thickness"]
    shaft_bottom = tab_top - p["joint_screw_length"]
    shaft = (cq.Workplane("XZ").workplane(offset=-shaft_bottom).center(x, z)
             .circle(shaft_r).extrude(-p["joint_screw_length"]))
    head = (cq.Workplane("XZ").workplane(offset=-tab_top).center(x, z)
            .circle(head_r).extrude(-head_h))
    screw = shaft.union(head)
    socket_dia = p["joint_screw_socket_af"] / math.cos(math.radians(30.0))
    socket = (cq.Workplane("XZ").workplane(offset=-(tab_top + head_h + 0.01)).center(x, z)
              .polygon(6, socket_dia).extrude(p["joint_screw_socket_depth"] + 0.01))
    return screw.cut(socket)


def motor(p, x, z):
    hub = (cq.Workplane("XZ").center(x, z).circle(p["motor_hub_radius"]).extrude(-p["motor_hub_height"]))
    can = (cq.Workplane("XZ").center(x, z).circle(p["motor_radius"]).extrude(-p["motor_height"]))
    return hub.union(can)


def _surface_z(solid, x, y, zlo=0.0, zhi=95.0):
    """Find the foremost (+Z) shell surface at (x, y), including hollow canopy walls."""
    step = 0.25
    outside_z = zhi
    z = zhi
    while z >= zlo:
        if solid.isInside(cq.Vector(x, y, z)):
            # Scan from the front until we enter the first solid wall; then bisect its front face.
            inside_z = z
            lo, hi = inside_z, outside_z
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                if solid.isInside(cq.Vector(x, y, mid)):
                    lo = mid
                else:
                    hi = mid
            return 0.5 * (lo + hi)
        outside_z = z
        z -= step
    return None


def led_slot(p, solid, fs=1.0):
    """Cutter = slit through the front shell + a pocket right behind it (dims given in final mm, fs = scale)."""
    import numpy as np
    y0 = p["led_y"]
    L, h, wall = p["led_slit_length"] / fs, p["led_slit_height"] / fs, p["led_wall"] / fs
    PL, PH, PD = p["led_pocket_length"] / fs, p["led_pocket_height"] / fs, p["led_pocket_depth"] / fs
    # the nose slopes back as it rises: take the most set-back shell point over the whole pocket footprint
    xs = np.linspace(-max(L, PL) / 2, max(L, PL) / 2, 9)
    ys = np.linspace(y0 - PH / 2, y0 + PH / 2, 5)
    # The wider pocket footprint can extend beyond the tapered canopy at its upper corners.
    # Use actual shell intersections only; fail only if the slit region has no front surface at all.
    zs = [_surface_z(solid, float(x), float(y)) for x in xs for y in ys]
    zs = [z for z in zs if z is not None]
    if not zs:
        raise RuntimeError("LED slit position does not intersect the front shell")
    zmax, zmin = max(zs), min(zs)
    zp = zmin - wall                                   # pocket front face
    slit = cq.Workplane("XY").box(L, h, (zmax + 1.0) - zp).translate((0, y0, (zmax + 1.0 + zp) / 2))
    pocket = cq.Workplane("XY").box(PL, PH, PD + 0.2).translate((0, y0, (zp + 0.2 + zp - PD) / 2))
    return slit.union(pocket)


def camera_parts(p, fs=1.0):
    """(housing, cutter): square lens housing to add outside, and lens hole + internal pocket to cut. fs = scale factor."""
    sz, rh = p["camera_block_size"] / fs, p["camera_lens_hole_radius"] / fs
    prot, wall = p["camera_block_protrusion"] / fs, p["camera_wall"] / fs
    w, h, d = p["camera_cavity_w"] / fs, p["camera_cavity_h"] / fs, p["camera_cavity_d"] / fs
    # local frame: origin = lens exit on the nose face, axis +Z = looking forward
    barrel = cq.Workplane("XY").workplane(offset=-1.0).rect(sz, sz).extrude(prot + 1.0)
    try:
        barrel = barrel.edges("|Z").fillet(p["camera_block_fillet"] / fs)
    except Exception:
        pass
    import math
    t = math.radians(p["camera_tilt_down_deg"])
    # the pocket's upper-front corner swings forward when tilted: push the pocket back so it keeps `wall`
    c = ((h / 2) * math.sin(t) + wall) / math.cos(t)
    hole = cq.Workplane("XY").workplane(offset=-c - 0.2).circle(rh).extrude(c + 0.2 + prot + 1.0)
    cavity = cq.Workplane("XY").box(w, h, d).translate((0, 0, -c - d / 2))
    cutter = hole.union(cavity)
    place = lambda wp: (wp.rotate((0, 0, 0), (1, 0, 0), p["camera_tilt_down_deg"])
                        .translate((0, p["camera_y"] / fs, p["body_front_z"])))
    return place(barrel), place(cutter)


def prop(p, x, z):
    blade = (cq.Workplane("XZ").workplane(offset=-p["prop_y"]).center(x, z)
             .rect(2 * p["prop_radius"], p["prop_chord"]).extrude(-p["prop_thickness"]))
    blade = blade.edges("|Y").fillet(min(p["prop_chord"], p["prop_thickness"] * 2) / 2.5)
    hub = (cq.Workplane("XZ").workplane(offset=-p["prop_y"] - p["prop_thickness"] / 2 + p["prop_hub_height"] / 2 - p["prop_hub_height"] / 2)
           .center(x, z).circle(p["prop_hub_radius"]).extrude(-p["prop_hub_height"]))
    return blade.union(hub)


def body_shell_parts(p):
    """Return the lower hull shell and hollow crown as independent solids.

    Build each component from its original source surfaces instead of cutting
    the already-fused body at the hull/canopy seam. No joining or locking
    features are added between the pieces.
    """
    if not (p.get("use_measured_canopy") and p.get("canopy_faceted")):
        # Preserve compatibility for alternate parameter configurations.
        full = body(p)
        split_y = float(p["body_height"]) + 0.2
        lower_box = cq.Workplane("XY").box(1000.0, split_y + 2.0, 1000.0).translate(
            (0, (split_y - 2.0) / 2.0, 0))
        crown_box = cq.Workplane("XY").box(1000.0, 100.0, 1000.0).translate(
            (0, split_y + 50.0, 0))
        return full.intersect(lower_box), full.intersect(crown_box)

    # Use the same rim vertices as the canopy so no crown edge falls inside the hull.
    pts = canopy_base_points(p)
    hull = cq.Workplane("XZ").polyline(pts).close().extrude(-p["body_height"])
    crown = faceted_canopy(p)

    wall = float(p.get("body_shell_thickness", 2.0))
    if wall <= 0 or wall >= min(p["body_half_width"], p["body_height"]):
        raise ValueError("body_shell_thickness must be positive and smaller than the body")
    cz = (p["body_front_z"] + p["body_rear_z"]) / 2.0
    scale_x = max(0.1, 1.0 - wall / p["body_half_width"])
    scale_z = max(0.1, 1.0 - wall / ((p["body_front_z"] - p["body_rear_z"]) / 2.0))
    inner_pts = [(x * scale_x, cz + (z - cz) * scale_z) for x, z in pts]
    lower_void = cq.Workplane("XZ").polyline(inner_pts).close().extrude(-(p["body_height"] + 2.0))
    lower_void = lower_void.translate((0, wall, 0))
    lower = hull.cut(lower_void)

    roof_void = faceted_canopy(p, inward=wall, open_bottom_y=p["body_height"] - 1.0,
                               roof_drop=wall)
    crown = crown.cut(roof_void)
    for name, part in (("body_lower", lower), ("body_crown", crown)):
        if not part.val().isValid() or len(part.val().Solids()) != 1:
            raise RuntimeError(f"{name} generated from source surfaces is invalid or not one solid")
    return lower, crown


def build(p=None):
    p = {**PARAMS, **(p or {})}
    inner_min = (min(p["guard_profile_inner"]) + p.get("guard_grow", 0.0)) if p.get("guard_use_profile") else p["guard_outer_radius"] - p["guard_wall"]
    assert p["prop_radius"] < inner_min, "prop would hit the guard"
    model = body(p)
    for i, (x, z) in enumerate(stations(p)):
        model = model.union(arm(p, x, z)).union(guard(p, x, z)).union(motor(p, x, z))
        if p["include_props"]:
            model = model.union(prop(p, x, z))
    f = 1.0
    if p.get("model_scale"):
        f = p["model_scale"]
    elif p.get("target_overall_mm"):
        bb = model.val().BoundingBox()
        f = p["target_overall_mm"] / max(bb.xlen, bb.zlen)
    if p["camera_enabled"]:
        barrel, cutter = camera_parts(p, f)
        model = model.union(barrel).cut(cutter)
    if p["led_enabled"]:
        model = model.cut(led_slot(p, model.val(), f))
    if f != 1.0:
        model = cq.Workplane("XY").newObject([model.val().scale(f)])
        print("uniform scale factor", round(f, 4))
    # Export convention: Z is vertical, the flat underside faces -Z, camera/front faces -Y.
    model = model.rotate((0, 0, 0), (1, 0, 0), 90)
    return model






def build_modular(p=None):
    """Build the split body, four removable arms, vertical screws, nut placeholders and props."""
    import math
    p = {**PARAMS, **(p or {})}
    inner_min = (min(p["guard_profile_inner"]) + p.get("guard_grow", 0.0)) if p.get("guard_use_profile") else p["guard_outer_radius"] - p["guard_wall"]
    assert p["prop_radius"] < inner_min, "prop would hit the guard"
    start_r = p["joint_arm_start_radius"]
    if start_r >= p["joint_entry_end_radius"]:
        raise ValueError("arm root must meet the side-wall entry opening")
    if len(p["joint_bolt_radii"]) != 2 or p["joint_bolt_radii"][0] >= p["joint_bolt_radii"][1]:
        raise ValueError("each arm requires two ordered floor-fastener stations")
    stations_list = stations(p)
    body_part, crown_part = body_shell_parts(p)
    # Add low floor rails, open the side walls (not the floor), then cut the matching nut traps.
    for x, z in stations_list:
        body_part = body_part.union(floor_mounting_rail(p, x, z))
    for x, z in stations_list:
        body_part = body_part.cut(arm_entry_slot(p, x, z))
    arm_parts, prop_parts, screw_parts, nut_parts = [], [], [], []
    for arm_index, (x, z) in enumerate(stations_list, 1):
        angle = math.atan2(z, x)
        module = arm(p, x, z, start_r=start_r, y_offset=p["body_shell_thickness"])
        module = module.union(mounting_tab(p, x, z))
        module = module.union(guard(p, x, z)).union(motor(p, x, z))
        arm_parts.append(module)
        for bolt_index, radius in enumerate(p["joint_bolt_radii"], 1):
            hx, hz = radius * math.cos(angle), radius * math.sin(angle)
            nut_cutter = (cq.Workplane("XZ").workplane(offset=-(p["joint_tab_y"] - p["joint_nut_depth"]))
                          .center(hx, hz).polygon(6, p["joint_nut_af"] / math.cos(math.radians(30.0)))
                          .extrude(-p["joint_nut_depth"]))
            body_part = body_part.cut(nut_cutter)
            part_index = (arm_index - 1) * len(p["joint_bolt_radii"]) + bolt_index
            screw_parts.append((f"screw_{part_index}", screw_part(p, hx, hz)))
            nut_parts.append((f"nut_{part_index}", nut_part(p, hx, hz)))
        if p["include_props"]:
            prop_parts.append(prop(p, x, z))
    raw_shapes = [body_part.val()] + [s.val() for s in arm_parts] + [s.val() for s in prop_parts]
    raw_compound = cq.Compound.makeCompound(raw_shapes)
    bb = raw_compound.BoundingBox()
    if p.get("model_scale"):
        scale = p["model_scale"]
    elif p.get("target_overall_mm"):
        scale = p["target_overall_mm"] / max(bb.xlen, bb.zlen)
    else:
        scale = 1.0
    # Camera and LED features belong to the body only; dimensions remain final-mm based.
    if p["camera_enabled"]:
        barrel, cutter = camera_parts(p, scale)
        body_part = body_part.union(barrel).cut(cutter)
    if p["led_enabled"]:
        # The LED slit sits above the hull/crown split, so keep it on the crown piece.
        crown_part = crown_part.cut(led_slot(p, crown_part.val(), scale))
    parts = {"body_lower": body_part, "body_crown": crown_part}
    for i, part in enumerate(arm_parts, 1):
        parts[f"arm_{i}"] = part
    for i, part in enumerate(prop_parts, 1):
        parts[f"prop_{i}"] = part
    for name, part in screw_parts:
        parts[name] = part
    for name, part in nut_parts:
        parts[name] = part
    if scale != 1.0:
        parts = {name: cq.Workplane("XY").newObject([shape.val().scale(scale)])
                 for name, shape in parts.items()}
        print("uniform scale factor", round(scale, 5))
    # Apply one rigid transform to every component so assembly and individual exports agree.
    # Old +Y (up) -> new +Z; old +Z (camera/front) -> new -Y; old -Y (underside) -> new -Z.
    parts = {name: part.rotate((0, 0, 0), (1, 0, 0), 90)
             for name, part in parts.items()}
    for name, part in parts.items():
        if not part.val().isValid():
            raise RuntimeError(f"Invalid CAD part: {name}")
        solid_count = len(part.val().Solids())
        print(f"part {name}: valid=True solids={solid_count}")
        if name.startswith("arm_") and solid_count != 1:
            raise RuntimeError(f"{name} is not a single connected solid ({solid_count} solids)")
    # Check that arm modules clear the lower shell and removable crown in the assembled pose.
    for arm_index, module in enumerate(arm_parts, 1):
        lower_intersection = body_part.val().intersect(module.val())
        crown_intersection = crown_part.val().intersect(module.val())
        lower_overlap = lower_intersection.Volume()
        crown_overlap = crown_intersection.Volume()
        lb, cb = lower_intersection.BoundingBox(), crown_intersection.BoundingBox()
        print(f"arm_{arm_index} clearance check: lower_overlap={lower_overlap:.6f}, "
              f"lower_bbox=({lb.xmin:.3f},{lb.ymin:.3f},{lb.zmin:.3f})-({lb.xmax:.3f},{lb.ymax:.3f},{lb.zmax:.3f}), "
              f"crown_overlap={crown_overlap:.6f}, "
              f"crown_bbox=({cb.xmin:.3f},{cb.ymin:.3f},{cb.zmin:.3f})-({cb.xmax:.3f},{cb.ymax:.3f},{cb.zmax:.3f})")
        if lower_overlap > 1e-4 or crown_overlap > 1e-4:
            raise RuntimeError(
                f"arm_{arm_index} interferes with body/crown: "
                f"lower={lower_overlap:.6f}, crown={crown_overlap:.6f}"
            )
    return parts, scale


def export_modular(parts, out_dir):
    """Export a named STEP assembly plus a compound STL and each separate component."""
    import os, json
    os.makedirs(out_dir, exist_ok=True)

    # STEP must be a real CadQuery Assembly, not a Workplane wrapping a Compound.
    # A compound can look like one selectable object in some CAD viewers; named
    # assembly children preserve body_lower and body_crown as separately selectable parts.
    assembly = cq.Assembly(name="drone_parametric")
    for name, part in parts.items():
        assembly.add(part.val(), name=name)
    assembly.save(os.path.join(out_dir, "drone_parametric.step"), exportType="STEP")

    # Keep the convenient whole-model STL as a compound mesh.
    compound = cq.Compound.makeCompound([part.val() for part in parts.values()])
    cq.exporters.export(cq.Workplane("XY").newObject([compound]),
                        os.path.join(out_dir, "drone_parametric.stl"),
                        tolerance=0.05, angularTolerance=0.1)
    try:
        import trimesh
        mesh = trimesh.load(os.path.join(out_dir, "drone_parametric.stl"))
        if not mesh.is_watertight:
            trimesh.repair.fill_holes(mesh)
            trimesh.repair.fix_normals(mesh)
            mesh.export(os.path.join(out_dir, "drone_parametric.stl"))
    except Exception as e:
        print("Assembly STL repair skipped:", e)

    for name, part in parts.items():
        export(part, out_dir, name)
    manifest = {
        "assembly": "drone_parametric.step",
        "branch_scope": "demo",
        "export_orientation": {
            "vertical_axis": "+Z",
            "camera_front_direction": "-Y",
            "flat_underside_direction": "-Z",
            "transform_from_design_coordinates": "(x, y, z) -> (x, -z, y)",
            "rotation_degrees_about_X": 90
        },
        "parts": [
            {"name": name, "step": f"{name}.step", "stl": f"{name}.stl",
             "role": ("lower body shell with reinforced floor rails, side entry openings and hex-nut traps" if name == "body_lower"
                      else "separate removable upper crown/canopy" if name == "body_crown"
                      else "removable arm module with an integral floor-mounting tongue, guard and motor" if name.startswith("arm_")
                      else "separate propeller" if name.startswith("prop_")
                      else "simplified vertical socket-head screw placeholder; use real M3 hardware" if name.startswith("screw_")
                      else "simplified hex-nut placeholder; replace with a real metal M3 nut" if name.startswith("nut_")
                      else "component")}
            for name in parts
        ],
        "joint": {
            "type": "integral arm tongue seated on reinforced inner-floor rails",
            "fastener": "two vertical M3 screws per arm, accessed from above after removing the crown",
            "fastener_count": 8,
            "fastener_axis_design_coordinates": "Y (vertical); screws installed top-down",
            "fastener_axis_export_coordinates": "Z (vertical)",
            "nut_retention": "hexagonal nut traps in reinforced floor rails; metal nuts are required",
            "side_wall_fasteners": False,
            "floor_plate_cut_through": False,
            "arm_entry_openings": 4,
            "integral_tab": True,
            "bolt_radii_model_units": PARAMS["joint_bolt_radii"],
            "tab_width_model_units": PARAMS["joint_tab_width"],
            "tab_thickness_model_units": PARAMS["joint_tab_thickness"],
            "floor_rail_height_model_units": PARAMS["joint_floor_pad_height"],
            "approx_tab_thickness_mm": round(PARAMS["joint_tab_thickness"] * PARAMS["model_scale"], 3),
            "approx_m3_clearance_hole_diameter_mm": round(2 * PARAMS["joint_bolt_hole_radius"] * PARAMS["model_scale"], 3),
            "hardware_geometry_is_simplified": True,
            "physical_fit_tested": False,
            "note": "The CAD includes simplified screw and nut envelopes for assembly visualization. Use real M3 screws and metal nuts; print-fit, tool access, and flight-load testing are still required."
        }
    }
    with open(os.path.join(out_dir, "assembly-manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)


def write_validation_report(parts, scale, out_dir, params):
    """Write a report from the freshly generated in-memory CAD shapes."""
    import os, json
    compound = cq.Compound.makeCompound([part.val() for part in parts.values()])
    bb = compound.BoundingBox()
    components = {}
    for name, part in parts.items():
        shape = part.val()
        components[name] = {
            "shape_valid": bool(shape.isValid()),
            "solid_count": len(shape.Solids())
        }
    p = {**PARAMS, **params}
    report = {
        "report_version": 3,
        "generated_from": "cad/parametric/drone_parametric.py",
        "branch": "demo",
        "validation_scope": "Fresh CadQuery in-memory geometry checks and workflow output-presence checks; no physical print-fit test",
        "assembly": {
            "shape_valid": bool(compound.isValid()),
            "solid_count": len(compound.Solids()),
            "bbox_mm": [round(bb.xlen, 6), round(bb.ylen, 6), round(bb.zlen, 6)]
        },
        "components": components,
        "joint": {
            "type": "integral floor-mounted tongue with vertical fasteners",
            "model_scale": scale,
            "fastener_count": 8,
            "fasteners_per_arm": len(p["joint_bolt_radii"]),
            "bolt_centres_model_units": p["joint_bolt_radii"],
            "bolt_clearance_hole_diameter_mm": round(2 * p["joint_bolt_hole_radius"] * scale, 3),
            "tab_thickness_mm": round(p["joint_tab_thickness"] * scale, 3),
            "floor_rail_height_mm": round(p["joint_floor_pad_height"] * scale, 3),
            "nut_trap_depth_mm": round(p["joint_nut_depth"] * scale, 3),
            "screws_vertical_and_top_accessible": True,
            "nut_traps_in_floor_rails": True,
            "side_wall_fasteners": False,
            "floor_plate_cut_through": False,
            "hole_centres_aligned_by_shared_coordinates": True,
            "arm_tab_is_integral": True,
            "hardware_geometry_is_simplified": True,
            "physical_fit_tested": False
        },
        "stl_watertight": "not_checked",
        "step_round_trip_import": "not_checked",
        "known_limitations": [
            "CAD validity checks apply to in-memory CadQuery shapes before export.",
            "The workflow checks that exported STEP/STL files exist and are non-empty; STEP round-trip import is not yet checked.",
            "STL watertightness is not reported by this validation step.",
            "Screw and nut solids are simplified envelopes without threads; use real metal M3 hardware.",
            "The mounting-hole coordinates are shared by the tabs and nut traps, but physical print-fit, screwdriver access in the printed body, and flight-load strength still require testing."
        ]
    }
    if not report["assembly"]["shape_valid"]:
        raise RuntimeError("Generated assembly compound is invalid")
    if report["assembly"]["solid_count"] != len(parts):
        raise RuntimeError(f"Assembly has {report['assembly']['solid_count']} solids; expected {len(parts)}")
    if any(not v["shape_valid"] or v["solid_count"] != 1 for v in components.values()):
        raise RuntimeError("One or more exported components is invalid or not a single solid")
    with open(os.path.join(out_dir, "validation-report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


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


if __name__ == "__main__":
    params = {}
    if len(sys.argv) > 2 and sys.argv[1] == "--params":
        params = json.load(open(sys.argv[2], encoding="utf-8"))
    parts, scale = build_modular(params)
    compound = cq.Compound.makeCompound([part.val() for part in parts.values()])
    bb = compound.BoundingBox()
    print("assembly bbox", bb.xlen, bb.ylen, bb.zlen, "valid", compound.isValid(),
          "solids", len(compound.Solids()))
    out = "output/قالب گیری شده/smooth2-parametric"
    if not compound.isValid() or len(compound.Solids()) != len(parts):
        raise RuntimeError("Generated assembly failed validity/solid-count checks")
    export_modular(parts, out)
    write_validation_report(parts, scale, out, params)
    with open(out + "/drone_parametric.params.json", "w", encoding="utf-8") as f:
        json.dump({**PARAMS, **params}, f, indent=2, ensure_ascii=False)

# Modular export: lower body shell, separate crown, four removable arm/guard/motor modules,
# and separate propellers. The crown-to-lower-body joining method is intentionally undecided.
# Regeneration is restricted to the demo branch; no crown-to-base joining features are included yet.
