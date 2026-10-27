"""Validate drone_parametric output against the smooth 2 source mesh -> validation-report.json"""
import sys, json, numpy as np, trimesh
sys.path.insert(0, "cad/parametric")
import drone_parametric as d
OUT = "output/قالب گیری شده/smooth2-parametric"
v = d.build().val()
b = trimesh.load(OUT + "/drone_parametric.stl")
a = trimesh.load("output/قالب گیری شده/smooth 2/quadcopter_smooth_2_reduced.stl"); a.merge_vertices()
rep = {"solids": len(v.Solids()), "shape_valid": bool(v.isValid()), "stl_watertight": bool(b.is_watertight),
       "volume_mm3": round(v.Volume(), 1), "source_volume_mm3": round(float(a.volume), 1),
       "extents_mm": b.extents.round(2).tolist(), "source_extents_mm": a.extents.round(2).tolist()}
# surface distance between new model and source (sampled points, nearest-surface distance)
pts = b.sample(20000)
_, dist, _ = trimesh.proximity.closest_point(a, pts)
rep["new_to_source_surface_distance_mm"] = {"mean": round(float(dist.mean()), 2), "median": round(float(np.median(dist)), 2), "p95": round(float(np.percentile(dist, 95)), 2)}
pts2 = a.sample(20000)
_, dist2, _ = trimesh.proximity.closest_point(b, pts2)
rep["source_to_new_surface_distance_mm"] = {"mean": round(float(dist2.mean()), 2), "median": round(float(np.median(dist2)), 2), "p95": round(float(np.percentile(dist2, 95)), 2)}
s = b.section(plane_origin=[0, 9, 0], plane_normal=[0, 1, 0]); sp = np.vstack(s.discrete); rows = []
for (cx, cz) in d.stations(d.PARAMS):
    r = np.hypot(sp[:, 0] - cx, sp[:, 2] - cz); m = (r > 33) & (r < 38) & ((sp[:, 0] - cx) * np.sign(cx) > 0); o = r[m]
    rows.append({"center": [cx, cz], "outer_R_min": round(float(o.min()), 3), "outer_R_max": round(float(o.max()), 3), "roundness_error_mm": round(float(o.max() - o.min()), 3)})
rep["guards_outer_roundness_at_y9"] = rows
rep["known_differences"] = ["Original guard rings are slightly egg-shaped on the body side; new rings are exact circles by request.",
    "Original propellers are noisy curved multi-blade shapes inside the mesh; new ones are simple 2-blade placeholders.",
    "Original motor/arm interior is a noisy blob; new arms/motors are clean primitives.",
    "Lower hull is parametric (polygon); canopy follows measured sections of the original."]
json.dump(rep, open(OUT + "/validation-report.json", "w"), indent=2, ensure_ascii=False)
print(json.dumps(rep, indent=1, ensure_ascii=False))
