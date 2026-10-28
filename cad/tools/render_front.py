"""Close-up preview of the front (camera + LED chevron). Usage: render_front.py out.png"""
import sys, numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUT = "output/قالب گیری شده/smooth2-parametric/"
m = trimesh.load(OUT + "drone_parametric.stl")
def draw(ax, mesh, color, ec=0.12, z=1):
    V = mesh.vertices[:, [0, 2, 1]]; tri = V[mesh.faces]; n = mesh.face_normals[:, [0, 2, 1]]
    l = np.array([0.3, 0.5, 0.8]); l /= np.linalg.norm(l); sh = np.clip(n @ l, 0, 1) * .75 + .25
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.c_[sh * color[0], sh * color[1], sh * color[2], np.ones(len(sh))], edgecolors=(0, 0, 0, ec), linewidths=.1, zorder=z))
fig = plt.figure(figsize=(18, 7))
for i, (el, az) in enumerate([(18, 70), (12, 90), (30, 115)]):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d"); ax.computed_zorder = False
    draw(ax, m, (.55, .58, .62)); pass
    ax.set_xlim(-35, 35); ax.set_ylim(35, 85); ax.set_zlim(-3, 27); ax.set_box_aspect((1, .7, .45)); ax.view_init(el, az); ax.set_axis_off()
fig.savefig(sys.argv[1], dpi=70, bbox_inches="tight")
