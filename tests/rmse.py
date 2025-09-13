import open3d as o3d
import numpy as np
from collections import defaultdict
import os
import matplotlib.pyplot as plt

# Colores ANSI para terminal
RED = "\033[91m"
GREEN = "\033[92m"
ENDC = "\033[0m"

# ========= CONFIGURACIÓN =========
voxel_size = 0.07

file_gt = "college_gt/new-college-29-01-2020-1cm-resolution-1stSection.ply"
file_pred = "college/mesh_college.stl"

# file_gt = "cow_gt/cow_gt.ply"
# file_pred = "cow/mesh_cow_big.stl"

sample_points = 1_000_000

### COLLEGE
manual_transform = np.array([
    [ 3.61624570e-01,  9.32323801e-01,  0.00000000e+00, -2.28440089e-01],
    [-9.32301081e-01,  3.61615758e-01, -6.98126030e-03,  6.03427987e+00],
    [-6.50879514e-03,  2.52459525e-03,  9.99975631e-01,  1.96256239e+00],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
])

### COW
# manual_transform = np.array([
#     [ 0.98768834,  0.15643447,  0.        ,  0.05663477],
#     [-0.15643447,  0.98768834,  0.        ,  0.26270512],
#     [ 0.        ,  0.        ,  1.        ,  0.36      ],
#     [ 0.        ,  0.        ,  0.        ,  1.        ]
# ])

# ========= FUNCIONES =========

def print_bounds(points, label):
    min_vals = points.min(axis=0)
    max_vals = points.max(axis=0)
    print(f"\n - Bounding box de {label}:")
    print(f"   X: {min_vals[0]:.2f} → {max_vals[0]:.2f}")
    print(f"   Y: {min_vals[1]:.2f} → {max_vals[1]:.2f}")
    print(f"   Z: {min_vals[2]:.2f} → {max_vals[2]:.2f}")

def refine_with_icp(pcd_pred, pcd_gt, voxel_size):
    threshold = voxel_size * 25 #1.5
    print(f"\nEjecutando ICP con threshold = {threshold:.2f} m ...")

    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd_pred, pcd_gt, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    print("✅ ICP completado.")
    print("Matriz refinada:\n", reg_p2p.transformation)
    pcd_pred.transform(reg_p2p.transformation)
    return pcd_pred

def voxel_centroids(pcd, voxel_size):
    points = np.asarray(pcd.points)
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)

    voxel_dict = defaultdict(list)
    for i, voxel in enumerate(voxel_indices):
        key = tuple(voxel)
        voxel_dict[key].append(points[i])

    centroids = {k: np.mean(v, axis=0) for k, v in voxel_dict.items()}
    return centroids

def compute_voxel_errors(centroids_gt, centroids_pred):
    common_keys = set(centroids_gt.keys()).intersection(centroids_pred.keys())
    if not common_keys:
        print(f"{RED}No se encontraron voxeles comunes.{ENDC}")
        return None

    errors = []
    error_points = []
    for key in common_keys:
        diff = centroids_gt[key] - centroids_pred[key]
        dist = np.linalg.norm(diff)
        errors.append(dist)
        error_points.append(centroids_gt[key])

    rmse = np.sqrt(np.mean(np.square(errors)))
    return rmse, error_points, errors, len(common_keys)

def visualize_errors(points, errors):
    norm_errors = (errors - np.min(errors)) / (np.ptp(errors) + 1e-8)
    colors = plt.get_cmap("jet")(norm_errors)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd], window_name="Error por Voxel")

# ========= CARGA DE NUBES =========

pcd_gt = o3d.io.read_point_cloud(file_gt)
print_bounds(np.asarray(pcd_gt.points), file_gt)

mesh = o3d.io.read_triangle_mesh(file_pred)
mesh.compute_vertex_normals()
mesh.transform(manual_transform)
pcd_pred = mesh.sample_points_uniformly(number_of_points=sample_points)
print_bounds(np.asarray(pcd_pred.points), file_pred)

# Refinar con ICP
pcd_pred = refine_with_icp(pcd_pred, pcd_gt, voxel_size)

# ========= CÁLCULO DE RMSE =========

centroids_gt = voxel_centroids(pcd_gt, voxel_size)
centroids_pred = voxel_centroids(pcd_pred, voxel_size)

result = compute_voxel_errors(centroids_gt, centroids_pred)

if result:
    rmse, points, errors, n = result
    print(f"\nVoxel Size: {GREEN}{voxel_size} m{ENDC}")
    print(f"Voxeles comparados: {GREEN}{n}{ENDC}")
    print(f"RMSE por voxel: {GREEN}{rmse:.4f} m{ENDC}")
    visualize_errors(points, np.array(errors))
else:
    print(f"{RED}\nNo se pudo calcular RMSE ni visualizar errores.{ENDC}")
