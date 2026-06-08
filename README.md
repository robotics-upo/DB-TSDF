<h1 align="center">DB-TSDF: Directional Bitmask-based Truncated Signed Distance Fields for Efficient Volumetric Mapping</h1>

<h3 align="center">
  Accepted at the <a href="https://2026.ieee-icra.org/">IEEE International Conference on Robotics and Automation (ICRA) 2026</a>
</h3>

<p align="center">
  <a href="https://robotics-upo.github.io/DB-TSDF/"><img src="https://img.shields.io/badge/Project-Website-007acc?style=flat" alt="Project Website"></a>
  <a href="https://youtu.be/yqLLAqjWyFM"><img src="https://img.shields.io/badge/YouTube-Video-red?style=flat&logo=youtube" alt="YouTube Video"></a>
  <a href="https://arxiv.org/abs/2509.20081"><img src="https://img.shields.io/badge/arXiv-Paper-b31b1b?style=flat&logo=arxiv" alt="arXiv Paper"></a>
</p>


**DB-TSDF** presents a high-efficiency, CPU-only framework for volumetric mapping. It utilizes a novel directional bitmask-based integration scheme to incrementally fuse LiDAR data into a dense voxel grid.

Key features include:
- **Directional Kernels:** Efficiently model beam geometry and occlusion in 3D.
- **Bitmask Encoding:** Ensures constant-time updates per scan, independent of grid resolution.
- **High Performance:** Multi-threaded C++ implementation fully integrated with ROS 2.

The design prioritizes predictable runtime and high-resolution reconstruction, making it an ideal solution for robotic platforms with limited GPU resources. 

![Example reconstruction](docs/media/college_tittle.png)




## 1. Prerequisites

Before you begin, make sure you have ROS 2 Humble and Ubuntu 22.04 (or higher) installed on your system. These are the core requirements for the project to run smoothly. If you haven't installed ROS 2 Humble yet, follow the official [installation guide](https://docs.ros.org/en/humble/Installation.html) for your platform. This guide will walk you through all the necessary steps to set up the core ROS 2 environment on your system. 


 

## 2. Installation

### Option A — Docker (recommended)
The `Dockerfile` is self-contained: it installs ROS 2 Humble and every
dependency, clones the repo and builds it with `colcon`. Fastest way to a clean,
reproducible setup.

   ```bash
   git clone https://github.com/robotics-upo/DB-TSDF.git
   cd DB-TSDF
   docker build -t db_tsdf_ros2:humble \
     --build-arg USER_UID=$(id -u) \
     --build-arg USER_GID=$(id -g) .
   xhost +local:docker   # allow GUI apps like RViz
   docker run -it \
     --env="DISPLAY" \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     --name db_tsdf_container \
     db_tsdf_ros2:humble
   ```

To resume the same container later: `docker start -ai db_tsdf_container`.

### Option B — Local ROS 2 workspace
Clone into the `src` folder of a ROS 2 workspace and build with `colcon`:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/robotics-upo/DB-TSDF.git
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

 

## 3. Running the Code

The launch system uses a single main launch file (`mapper_launch.py`) and a config argument that selects which dataset configuration to load (e.g., `mai` loads `mai.yaml` and `mai.rviz`).

### Quick start with a test dataset

1. Download a ready-to-play MaiCity sequence. The script fetches the official archive, converts it from ROS 1 to rosbag2 and removes every intermediate file, leaving the dataset at `datasets/mai_city/<sequence>/` (path printed on completion):
   ```bash
   ./src/db_tsdf/scripts/download_test.sh        # sequence 01 only — quick test (~215 MB)
   ./src/db_tsdf/scripts/download_mai_city.sh    # all sequences (~3.4 GB)
   ```

2. Launch DB-TSDF with the matching config. RViz will open and the node will wait for data:
   ```bash
   ros2 launch db_tsdf mapper_launch.py config:=mai
   ```

3. In a second terminal, play the dataset back:

   For the Docker container, open one with:
   ```bash
   docker exec -it db_tsdf_container bash
   ```

   and run it with:
   ```bash
   ros2 bag play ~/ros2_ws/datasets/mai_city/01
   ```


 


## 4. Configuration

The system is highly configurable via YAML parameters. (e.g., `config/college.yaml`).

### Core Parameters
| Parameter | Type | Description | Default |
| :--- | :---: | :--- | :---: |
| `in_cloud` | `string` | Input PointCloud2 topic | `/os_cloud_node/points` |
| `odom_frame_id` | `string` | Fixed frame for TF lookup | `odom` |
| `use_tf` | `bool` | Enable/Disable TF transformations | `True` |
| `verbose_init` | `bool` | Full parameter dump + kernel preview at startup | `False` |

### Grid Definition
| Parameter | Type | Description | Default |
| :--- | :---: | :--- | :---: |
| `tdf_grid_res` | `float` | Voxel side length in meters | `0.05` |
| `tdf_max_cells` | `int` | Max active cells in hash table | `75000` |
| `tdfGridSizeX/Y*_low/high` | `float` | Horizontal volume boundaries | `+/-100` |
| `tdfGridSizeZ_low/high` | `float` | Vertical volume boundaries | `-10 / +50` |

### Integration Kernel
| Parameter | Type | Description | Default |
| :--- | :---: | :--- | :---: |
| `kernel_size` | `int` | Kernel size (odd number) | `7` |
| `bins_az` / `bins_el` | `int` | Angular discretization | `60` |
| `occ_min_hits` | `int` | Min measurements to mark occupied | `50` |




## 5. Output Data and Services

The node provides four `std_srvs/srv/Trigger` services to export the reconstructed map. Each one runs in the background and writes its output relative to the directory the node was launched from:

| Service | Output | Description |
| :--- | :--- | :--- |
| `/save_grid_pcd` | `grid_data.pcd` | Occupied-voxel point cloud (PCD) |
| `/save_grid_ply` | `grid_data.ply` | Occupied-voxel point cloud (PLY) |
| `/save_grid_csv` | `grid_data_csv/` | Per-cell voxel data (CSV + PLY), one file pair per allocated subgrid cell |
| `/save_grid_mesh` | `mesh.stl` | Surface mesh extracted with Marching Cubes |

```bash
ros2 service call /save_grid_pcd  std_srvs/srv/Trigger "{}"
ros2 service call /save_grid_ply  std_srvs/srv/Trigger "{}"
ros2 service call /save_grid_csv  std_srvs/srv/Trigger "{}"
ros2 service call /save_grid_mesh std_srvs/srv/Trigger "{}"
```

 

<p align="right">(<a href="#readme-top">back to top</a>)</p>
## Citation
If you use DB-TSDF in your research, please cite our paper:

```bibtex
@inproceedings{maese2026dbtsdf,
  title={DB-TSDF: Directional Bitmask-based Truncated Signed Distance Fields for Efficient Volumetric Mapping},
  author={Maese, Jose E. and Caballero, Fernando and Merino, Luis},
  booktitle={2026 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2026}
}
```

 


## Acknowledgements

![Logos](docs/media/fondos_proyectos.png)

This work was supported by the grants PICRA 4.0 (PLEC2023-010353), funded by the Spanish Ministry of Science and Innovation and the Spanish Research Agency (MCIN/AEI/10.13039/501100011033); and INSERTION (PID2021-127648OB-C31), funded by the "Agencia Estatal de Investigación - Ministerio de Ciencia, Innovación y Universidades" and the "European Union NextGenerationEU/PRTR".
