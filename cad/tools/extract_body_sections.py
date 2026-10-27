"""Extract the central-body plan sections from the 'smooth 2' mesh (rings/arms' discs removed).
Writes cad/parametric/body_sections.json: {y: [[x,z],...]} with a fixed number of points per level
so that CadQuery can ruled-loft them. The numbers are plain data: edit them to reshape the body."""
import json, sys, numpy as np, trimesh
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union

SRC = "output/قالب گیری شده/smooth 2/quadcopter_smooth_2_reduced.stl"
OUT = "cad/parametric/body_sections.json"
LEVELS = [18, 21, 23.5]      # only the clean canopy levels; the lower hull stays parametric (see drone_parametric.py)
N = 56
CENTERS = [(37, 39.5), (37, -39.5), (-37, 39.5), (-37, -39.5)]
DISC_R = 36.5

m = trimesh.load(SRC); m.merge_vertices()
discs = unary_union([Point(c).buffer(DISC_R, 64) for c in CENTERS])
# keep zone: the central body strip and the thin cross-neck between front and rear rings,
# so the round rings of the new model do not eat into (or double up with) the body
KEEP = unary_union([box(-26, -60, 26, 72), box(-45, -3.5, 45, 3.5)])

def polygon_from_section(sec):
    polys = [Polygon(e[:, [0, 2]]).buffer(0) for e in sec.discrete if len(e) > 3]
    polys = sorted(polys, key=lambda p: -p.area)
    res = None
    for p in polys:                      # even-odd composition (outer loops minus inner loops)
        res = p if res is None else res.symmetric_difference(p)
    return res

def resample(poly, n):
    ring = poly.exterior
    # start at the max-z point, go counter-clockwise (x increasing at start side)
    if not ring.is_ccw:
        ring = type(ring)(list(ring.coords)[::-1])
    pts = np.array(ring.coords)[:-1]
    start = np.argmax(pts[:, 1]); pts = np.roll(pts, -start, axis=0)
    line = np.vstack([pts, pts[:1]])
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(line, axis=0).T))]
    t = np.linspace(0, seg[-1], n, endpoint=False)
    return np.c_[np.interp(t, seg, line[:, 0]), np.interp(t, seg, line[:, 1])]

out = {}
for y in LEVELS:
    sec = m.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0])
    poly = polygon_from_section(sec)
    body = unary_union([poly.intersection(KEEP), poly.difference(discs)]) if y < 17 else poly
    if body.geom_type != "Polygon":
        body = max(body.geoms, key=lambda g: g.area)
    body = body.buffer(0.8).buffer(-0.8).simplify(0.4)       # tidy tiny slivers
    if body.geom_type != "Polygon":
        body = max(body.geoms, key=lambda g: g.area)
    pts = resample(Polygon(body.exterior), N)
    out[str(y)] = np.round(pts, 2).tolist()
    print(y, "area", round(body.area), "bounds", [round(v, 1) for v in body.bounds])
json.dump({"levels": LEVELS, "points_per_level": N, "sections": out}, open(OUT, "w"))
