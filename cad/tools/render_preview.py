"""Quick shaded preview PNG of the generated STL (no GPU needed)."""
import sys, numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
src = sys.argv[1]; out = sys.argv[2]
views = [(35, -60), (22, 90), (60, -90)]
m = trimesh.load(src)
V = m.vertices[:, [0, 2, 1]]          # plot with Y(up) as matplotlib Z
tri = V[m.faces]; n = m.face_normals[:, [0, 2, 1]]
light = np.array([0.3, -0.5, 0.8]); light /= np.linalg.norm(light)
sh = np.clip(n @ light, 0, 1) * 0.75 + 0.25
cols = np.c_[sh * 0.55, sh * 0.58, sh * 0.62, np.ones(len(sh))]
fig = plt.figure(figsize=(18, 7))
for i, (el, az) in enumerate(views):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(tri, facecolors=cols, edgecolors=(0, 0, 0, 0.15), linewidths=0.1))
    ax.set_xlim(-80, 80); ax.set_ylim(-80, 80); ax.set_zlim(-5, 60); ax.set_box_aspect((1, 1, 0.4))
    ax.view_init(el, az); ax.set_axis_off()
fig.savefig(out, dpi=70, bbox_inches="tight")
