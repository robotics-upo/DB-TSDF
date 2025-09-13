# DB-TSDF: Directional Bitmask-based Truncated Signed Distance Fields for Efficient Volumetric Mapping

A lightweight **CPU-only volumetric mapping system** based on Truncated Signed Distance Fields (TSDF).  
It integrates LiDAR point clouds into a voxel grid using directional kernels and bitmask encoding, enabling **real-time 3D reconstruction** without GPU acceleration.

![Example reconstruction](media/college_tittle.png)

---

## Overview

This framework incrementally fuses LiDAR data into a dense voxel grid:  
- **Directional kernels** model LiDAR beam geometry and occlusion.  
- **Bitmask encoding** ensures constant-time updates per scan.  
- **Signed distance representation** differentiates free and occupied space.  
- **Multi-threaded C++** implementation inside ROS 2.  

The design emphasizes predictable runtime, high resolution, and full CPU compatibility, making it suitable for robotic platforms where GPU resources are limited.
