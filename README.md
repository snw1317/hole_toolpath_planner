# hole_toolpath_planner

`hole_toolpath_planner` is a ROS 2 Humble package that detects circular holes in triangle meshes and returns poses (and optional toolpaths) suitable for machining or inspection workflows.

## Prerequisites
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble desktop installation
- Build tools: `git`, `cmake`, `build-essential`
- ROS 2 dependencies: `ros-humble-pcl-ros`, `ros-humble-pcl-conversions`
- Python tooling (optional Open3D detector): `python3-pip`, `python3-open3d`

Install the base tooling if needed:

```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y ros-humble-desktop ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install -y python3-pip python3-open3d
# If python3-open3d is not available on your distribution, install via pip instead:
# python3 -m pip install --user open3d
```

## Clone & build
Create or reuse a ROS 2 workspace (example: `~/holes_pose_tpp_ws`) and clone the repository into the `src` folder:

```bash
mkdir -p ~/holes_pose_tpp_ws/src
cd ~/holes_pose_tpp_ws/src

git clone https://github.com/<your-org>/hole_toolpath_planner.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch the hole detector service
Two implementations are available once the workspace is built and sourced.

### C++ (PCL) detector
Launch the original C++ node:

```bash
ros2 launch hole_toolpath_planner hole_detector.launch.py
```

This starts the `detect_holes` service server (default name: `/detect_holes`). Keep this terminal open while sending requests from another terminal.

### Python (Open3D) detector
Launch the Open3D-backed Python node:

```bash
ros2 launch hole_toolpath_planner_py detector_py.launch.py
```

The Python node exposes the same `/detect_holes` interface and publishes the same visualization topics (`/holes`, `/hole_markers`).
It fits cylinders with Open3D RANSAC, reports diameters (the `hole.diameter` field carries the diameter in meters), and reads its parameters from `hole_toolpath_planner_py/cfg/default_params.yaml`.

## Send a detection request
Open a new terminal, source the workspace, and call the service with the desired parameters. Update the mesh path and thresholds for your geometry.

```bash
cd ~/holes_pose_tpp_ws
source install/setup.bash

ros2 service call /detect_holes hole_toolpath_planner/srv/DetectHoles \
  "{mesh_path: '/home/snw13/hole_toolpath_planner_ws/src/hole_toolpath_planner/test_parts/hole_test_plate_m.stl', \
     min_diameter: 0.006, \
     max_diameter: 0.012, \
     min_length: 0.002, \
     watertight_hint: true}"
```

The diameter fields are expressed in meters (e.g. `0.010` = 10 mm).

If the service returns an empty array, check the node's terminal for diagnostics describing why candidate holes were rejected.

## Next steps
- Adjust `cfg/default_params.yaml` (C++) or `hole_toolpath_planner_py/cfg/default_params.yaml` (Python) to tune sampling density and detection thresholds.
- Visualize results by enabling RViz markers in the launch file.
- Extend the planner with toolpath generation strategies under `src/toolpath/`.
