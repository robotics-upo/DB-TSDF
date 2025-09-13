import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Colores consola
RED = "\033[91m"
GREEN = "\033[92m"
ENDC = "\033[0m"

def load_mesh_as_pcd(file_path, num_points=1_000_000):
    mesh = o3d.io.read_triangle_mesh(file_path)
    mesh.compute_vertex_normals()
    return mesh.sample_points_uniformly(number_of_points=num_points)

def compute_colored_error_map(source, target):
    dists = np.asarray(source.compute_point_cloud_distance(target))
    max_dist = np.percentile(dists, 95)
    norm_dists = np.clip(dists / max_dist, 0.0, 1.0)
    colors = plt.get_cmap("plasma")(norm_dists)[:, :3]
    source.colors = o3d.utility.Vector3dVector(colors)
    return dists

def refine_with_icp(source, target, voxel_size):
    threshold = voxel_size * 25
    print(f"\nEjecutando ICP con threshold {threshold:.3f}...")
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    source.transform(result_icp.transformation)
    print(f"ICP completado.")
    return source

# ========================
# CONFIGURACIÓN
# ========================
# voxel_size = 0.01

# gt_path = "college_gt/new-college-29-01-2020-1cm-resolution-1stSection.ply"
# pred_path = "college/mesh_college.stl"

### COLLEGE
# T_manual = np.array([
#     [ 3.61624570e-01,  9.32323801e-01,  0.00000000e+00, -2.28440089e-01],
#     [-9.32301081e-01,  3.61615758e-01, -6.98126030e-03,  6.03427987e+00],
#     [-6.50879514e-03,  2.52459525e-03,  9.99975631e-01,  1.96256239e+00],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
# ])

voxel_size = 0.01

gt_path = "cow_gt/cow_gt.ply"
pred_path = "cow/mesh_cow.stl"

### COW
T_manual = np.array([
    [ 0.98768834,  0.15643447,  0.        ,  0.05663477],
    [-0.15643447,  0.98768834,  0.        ,  0.26270512],
    [ 0.        ,  0.        ,  1.        ,  0.36      ],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

# ========================
# CARGA Y TRANSFORMACIÓN
# ========================
pc_gt = o3d.io.read_point_cloud(gt_path)
pc_pred = load_mesh_as_pcd(pred_path)
pc_pred.transform(T_manual)

# Refinamiento opcional con ICP
pc_pred = refine_with_icp(pc_pred, pc_gt, voxel_size)

# ========================
# CÁLCULO DE CHAMFER DISTANCE
# ========================
dists_gt_to_pred = compute_colored_error_map(pc_gt, pc_pred)
dists_pred_to_gt = compute_colored_error_map(pc_pred, pc_gt)

mean_fwd = np.mean(dists_gt_to_pred)
mean_bwd = np.mean(dists_pred_to_gt)
chamfer_total = mean_fwd + mean_bwd

# ========================
# IMPRESIÓN DE RESULTADOS
# ========================
print(f"\nGT → Pred (mean): {GREEN}{mean_fwd:.4f} m{ENDC}")
print(f"Pred → GT (mean): {GREEN}{mean_bwd:.4f} m{ENDC}")
print(f"Chamfer Distance total: {GREEN}{chamfer_total:.4f} m{ENDC}")

# ========================
# VISUALIZACIÓN
# ========================
o3d.visualization.draw_geometries([pc_gt], window_name="GT → Pred (error)")
o3d.visualization.draw_geometries([pc_pred], window_name="Pred → GT (error)")

pc_gt.paint_uniform_color([0, 1, 0])   # Verde
pc_pred.paint_uniform_color([1, 0, 0]) # Rojo
o3d.visualization.draw_geometries([pc_gt, pc_pred], window_name="GT (verde) vs Pred (rojo)")
