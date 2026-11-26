
import numpy as np
import trimesh
import pyvista as pv

from pathlib import Path

MODEL = Path(__file__).resolve().parent / "models" / "rafale_style.glb"

# Load GLB model
scene = trimesh.load(str(MODEL), force="scene")

plotter = pv.Plotter()
plotter.set_background("#101827")

mesh_count = 0

for geometry in scene.dump():
    if not isinstance(geometry, trimesh.Trimesh):
        continue

    vertices = np.asarray(geometry.vertices)
    triangles = np.asarray(geometry.faces)

    if len(triangles) == 0:
        continue

    faces = np.column_stack([
        np.full(len(triangles), 3),
        triangles
    ]).ravel()

    mesh = pv.PolyData(vertices, faces)

    plotter.add_mesh(
        mesh,
        color="lightgray",
        smooth_shading=True,
        metallic=0.45,
        roughness=0.5
    )

    mesh_count += 1

print(f"Loaded {mesh_count} mesh components")

plotter.add_axes()
plotter.show_grid()
plotter.reset_camera()
plotter.show()
