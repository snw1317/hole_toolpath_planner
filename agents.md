# AGENTS.md — hole_toolpath_planner

Guide for human and AI contributors working on **hole_toolpath_planner**, a ROS 2 Humble package that detects circular holes in meshes (sheet surfaces and closed solids) and emits stable poses and optional toolpaths for machining/inspection.

---

## 1) Purpose & scope
- **Inputs**: Triangle mesh file (`.stl`, `.ply`) plus parameters (radius limits, thresholds, mode selection).
- **Outputs**:
  - `HoleArray` with one pose per detected hole.
  - Optional `Toolpath` messages for approach/plunge/circular or spiral passes.
- **Supported geometry**:
  1) **Surface mode** — sheet-like meshes with **open boundary loops** (circular apertures).
  2) **Solid mode** — watertight meshes with **cylindrical bores** (through/blind).
  3) **Auto mode** — runs both pipelines and deduplicates overlapping results.

**Pose convention (non‑negotiable):**
- **Origin**: the **rim center on the entry surface** (intersection of the hole axis with the entry plane).
- **+Z**: points **into the hole** (i.e., into the part/material).
- **X/Y**: formed by Gram–Schmidt from a stable seed to create a right‑handed, orthonormal frame.

---

## 2) Environment & dependencies
- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble
- **Compiler**: C++17 or newer
- **Libs**: PCL (`common`, `io`, `filters`, `segmentation`, `surface`), Eigen3, `pcl_conversions`, `pcl_ros`, `rclcpp`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2`, `tf2_eigen`

Install example:
```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y ros-humble-desktop ros-humble-pcl-ros ros-humble-pcl-conversions
# PCL ships with Ubuntu 22.04; Eigen is a ROS dependency.
```

---

## 3) Public interfaces

### 3.1 Messages
`msg/Hole.msg`
```ros
# hole_toolpath_planner/ Hole.msg
# Pose at entry rim center; +Z points into the part.
# kind identifies which pipeline produced the detection.

# enums
uint8 SURFACE_CIRCLE=0
uint8 CYLINDER=1

std_msgs/Header header
geometry_msgs/Pose pose        # origin=entry rim center, +Z into hole
float32 radius                 # meters
float32 length                 # meters (axial inlier extent; 0 if unknown in surface mode)
geometry_msgs/Vector3 axis     # duplicate of pose Z (unit)
int32 id
uint8 kind
```

`msg/HoleArray.msg`
```ros
std_msgs/Header header
hole_toolpath_planner/Hole[] holes
```

`msg/Toolpath.msg`
```ros
std_msgs/Header header
int32 hole_id
string strategy                 # "circle" | "spiral"
float32 stepdown                # meters
float32 feedrate                # m/s (document units)
geometry_msgs/PoseStamped[] poses
```

### 3.2 Service
`srv/DetectHoles.srv`
```ros
string mesh_path
float32 min_radius
float32 max_radius
float32 min_length
bool watertight_hint
---
hole_toolpath_planner/HoleArray holes
```

---

## 4) Parameters (cfg/default_params.yaml)
```yaml
detection:
  mode: "auto"                 # auto | surface | solid
  dedupe_angle_deg: 5.0
  dedupe_center_tol: 0.0005    # meters (0.5 mm)

sampling:
  strategy: "uniform_area"     # triangle-area weighted
  points: 150000
  seed: 42                     # deterministic sampling unless overridden

normals:
  k: 30                        # KNN for normal estimation

surface_circle:
  min_loop_vertices: 6
  circularity_rmse_thresh: 0.002      # meters (2 mm)
  plane_fit_method: "pca"             # or ransac
  outer_boundary_policy: "largest_by_area"  # ignore outer shell loop per component
  into_hint: [0.0, 0.0, 1.0]          # used if face normals ambiguous
  thickness: null                      # optional known thickness (meters); else length=0

cylinder_fit:
  distance_threshold: 0.0008
  normal_distance_weight: 0.1
  max_iterations: 10000
  min_inliers: 800
  radius_min: 0.001
  radius_max: 0.050
  extract_iteratively: true

pose:
  z_direction_rule: "into_part"   # enforce +Z into material
  x_seed: "world_x"               # fallback for Gram–Schmidt
  neighbor_radius_scale: 2.0       # r_nbr = clamp(scale*r, [1.2r, 3r])
  neighbor_slab_thickness: 0.002   # meters for entry-plane neighborhood

toolpath:
  enabled: true
  approach_offset: 0.010           # meters along -Z (pre-entry)
  strategy: "circle"               # or "spiral"
  stepdown: 0.002
  passes: 3
```

---

## 5) Directory layout
```
hole_toolpath_planner/
  package.xml
  CMakeLists.txt
  include/hole_toolpath_planner/
    mesh_io.hpp
    mesh_sampler.hpp
    normals.hpp
    cylinder_segmentation.hpp
    surface_loops.hpp
    hole_pose_builder.hpp
    toolpath_builder.hpp
    markers.hpp
    types.hpp
    utils.hpp
  src/
    hole_detector_node.cpp
    mesh_io.cpp
    mesh_sampler.cpp
    normals.cpp
    cylinder_segmentation.cpp
    surface_loops.cpp
    hole_pose_builder.cpp
    toolpath_builder.cpp
    markers.cpp
    utils.cpp
  msg/  srv/  cfg/  launch/
  README.md  AGENTS.md
```

---

## 6) Algorithms & invariants

### 6.1 Surface‑mode (circular boundary apertures)
1. **Boundary extraction**: build half‑edge adjacency; edges with a single incident triangle are **boundary**. Trace to ordered loops.
2. **Outer vs hole**: per connected surface component, the largest loop by signed area on its best‑fit plane is the **outer border**; all others are **hole candidates**.
3. **Plane & circle fit**:
   - Plane by **PCA**: normal is smallest‑eigenvector; plane point = vertex mean.
   - Project loop vertices to plane (2D) and fit a circle (Pratt/Taubin or RANSAC). Reject if RMSE > `circularity_rmse_thresh` or too few vertices.
4. **Pose**:
   - **Origin**: circle center in 3D plane.
   - **Z**: into the part — if triangle normals coherent, take average as **outward** and set `Z = -n_plane`. Else orient `Z` so `dot(Z, into_hint) > 0`.
   - Build `X,Y` via Gram–Schmidt; `length = thickness` if provided, else `0`.
5. **Emit**: one `Hole` per accepted loop; `kind=SURFACE_CIRCLE`.

### 6.2 Solid‑mode (cylindrical bores)
1. **Sampling**: triangle‑area weighted points from the full mesh.
2. **Normals**: `NormalEstimationOMP(k)`.
3. **RANSAC cylinder**: `SACMODEL_CYLINDER` from points+normals with radius limits and thresholds. If `extract_iteratively`, remove inliers and loop.
4. **Entry plane & axis direction**:
   - From cylinder coefficients, get axis `p0, d_raw` and radius `r`. Project inliers to get endpoints `c_min, c_max`.
   - For each end, crop a **ring neighborhood**: radial window `r_nbr`, axial slab `t_nbr`; fit a plane (PCA or RANSAC) with normal `n_end`.
   - **Choose entry** so **+Z goes into material**: score `s = d_raw · ( -n_end )`; pick the larger positive score. Flip axis if needed → `d_inward`.
   - **Origin refinement**: intersect axis line `L(t)=p0+t d_inward` with entry plane `(p−p_plane)·n_entry=0`. If ill‑conditioned, fallback to `c_entry`.
5. **Pose**: `Z=d_inward`; construct `X,Y` via Gram–Schmidt. `length = t_max - t_min` from inlier projections. `kind=CYLINDER`.

### 6.3 Deduplication (Auto mode)
- Two detections are considered the **same hole** if:
  - Angle between axes ≤ `dedupe_angle_deg`, and
  - Center distance ≤ `dedupe_center_tol`, and
  - |radius difference| ≤ max(0.5 mm, 2%).
- Prefer **CYLINDER** over **SURFACE_CIRCLE** when both exist for the same aperture (richer `length`).

### 6.4 Invariants (must always hold)
- Origin lies on the entry surface plane (within `2 * distance_threshold`).
- `Z` is unit length and points into the hole.
- Rotation is orthonormal and right‑handed.
- Units are **meters** throughout.

---

## 7) Toolpath generation
- **Approach**: `origin - approach_offset * Z` (pre‑entry).
- **Plunge**: along `+Z` into the part.
- **Strategies**:
  - `circle`: concentric circles in local XY at step‑downs (`stepdown`, `passes`).
  - `spiral`: single continuous spiral with depth parameterized vs angle.
- **Surface‑mode**: if `thickness` unknown, keep paths on the entry plane for deburr/chamfer; if provided, step down to that depth.

---

## 8) RViz markers
- SPHERE at origin (small radius)
- LINE along ±axis segment through origin
- TEXT_VIEW_FACING with `id` and `⌀`

---

## 9) Build & run
Assume workspace `~/ws_holes`.
```bash
mkdir -p ~/ws_holes/src && cd ~/ws_holes/src
# place the package here: hole_toolpath_planner/
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch hole_toolpath_planner hole_detector.launch.py

# Service call
ros2 service call /detect_holes hole_toolpath_planner/srv/DetectHoles \
"{mesh_path: '/home/user/parts/bracket.stl', min_radius: 0.003, max_radius: 0.015, min_length: 0.002, watertight_hint: true}"
```

---

## 10) Coding standards & guardrails
- C++17; prefer Eigen for math. Keep functions pure where possible.
- Robustness first: return empty arrays with `RCLCPP_WARN` on failures; never crash the node.
- Determinism: fixed RNG seed for sampling unless `sampling.seed` provided.
- Performance baseline: 150k samples, ≤8 holes in a few seconds on a modern laptop.
- **Do not change** pose convention without a major version bump.

---

## 11) Testing matrix
- **Synthetic sheets** with known circular cutouts (varied radii, noise, decimation).
- **Watertight solids** with through and blind bores at oblique angles.
- **Countersinks/counterbores**: ensure core cylinder still detected; origin stays at outer entry rim.
- **Stress**: tiny holes near `radius_min`, short bores, near‑parallel to world axes.

**Assertions**:
- `dot(Z, n_outward) < 0` near entry rim (Z into material).
- Origin distance to entry plane ≤ `2 * distance_threshold`.
- Radius error ≤ 2% (clean), center error ≤ 1 mm.
- Length within 5% of thickness for through‑holes.

---

## 12) Backlog & extensions
- Slot/oval detection (ellipse fit) and slot toolpaths.
- Counterbore/countersink modeling by cone/step segmentation.
- Python bindings via `pybind11`.
- Topic‑based streaming (subscribe to mesh or point cloud topics).
- JSON/YAML export of hole summaries.

---

## 13) Contributor task board
1. Scaffold package (msgs/srv/params/launch).
2. Implement `mesh_sampler` (area weighted sampling).
3. Implement `normals` (OMP KNN).
4. Implement **surface_loops** (boundary tracing, plane+circle fit).
5. Implement **cylinder_segmentation** (RANSAC + iterative extraction).
6. Implement **hole_pose_builder** (entry plane choice, axis flip, origin intersection, Gram–Schmidt).
7. Markers + toolpath builder.
8. Tests & example meshes under `test_data/`.

---

## 14) Mini prompt (for codegen agents)
> Implement `surface_loops.{hpp,cpp}` to: (a) construct half‑edge adjacency; (b) extract ordered boundary loops; (c) per component, filter outer loop by largest projected area; (d) fit plane by PCA; (e) fit circle in plane (Pratt/Taubin); (f) reject loops exceeding `circularity_rmse_thresh`; (g) compute pose where origin is circle center and +Z points into the part using face‑normal consensus or `into_hint`; (h) emit `Hole` with `kind=SURFACE_CIRCLE`. Include robust fallbacks and unit tests.

