#!/usr/bin/env python3
"""
extract_holes_open3d_meters.py

Use Open3D to extract hole poses (xyz + rpy) from an STL mesh.
Mesh units are in meters.

- Detects cylindrical hole wall regions
- Clusters them into individual holes
- Fits a cylinder axis via PCA
- Filters for 5 mm and 10 mm diameter holes (0.005 m, 0.010 m)
- Outputs hole origin (xyz) and orientation (rpy)

Usage:
    python extract_holes_open3d_meters.py input_model.stl --out holes.csv
"""

import argparse
import csv
import math
import os
import numpy as np
import open3d as o3d


def load_mesh(path: str) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(path)
    if mesh.is_empty():
        try:
            from stl import mesh as stl_mesh
        except ImportError as exc:
            raise RuntimeError(f"Mesh is empty or failed to load: {path}") from exc
        stl_data = stl_mesh.Mesh.from_file(path)
        vertices = stl_data.vectors.reshape(-1, 3)
        triangles = np.arange(vertices.shape[0]).reshape(-1, 3)
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.remove_duplicated_vertices()
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_non_manifold_edges()
        if mesh.is_empty():
            raise RuntimeError(f"Mesh is empty or failed to load: {path}")
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    return mesh


def get_triangle_centroids_and_normals(mesh: o3d.geometry.TriangleMesh):
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    norms = np.asarray(mesh.triangle_normals)
    centroids = verts[tris].mean(axis=1)
    return centroids, norms


def cluster_points_dbscan(points: np.ndarray, eps=0.002, min_points=30):
    """DBSCAN in meters. eps default = 2 mm."""
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    return labels


def fit_axis_pca(points: np.ndarray):
    mean = points.mean(axis=0)
    centered = points - mean
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    axis_dir_candidates = vt
    axis_dir = axis_dir_candidates[np.argmax(np.abs(axis_dir_candidates[:, 2]))]
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    return mean, axis_dir


def compute_cylinder_params_from_points(points: np.ndarray):
    axis_point, axis_dir_raw = fit_axis_pca(points)
    d = points - axis_point
    t = d @ axis_dir_raw
    axis_points = axis_point + np.outer(t, axis_dir_raw)
    radial_vecs = points - axis_points
    radius = float(np.median(np.linalg.norm(radial_vecs, axis=1)))
    entry = axis_point + axis_dir_raw * t.min()
    exit_ = axis_point + axis_dir_raw * t.max()
    length = float(np.linalg.norm(exit_ - entry))
    axis_dir = (exit_ - entry) / np.linalg.norm(exit_ - entry)
    return radius, length, entry, exit_, axis_dir


def build_rotation_from_axis(axis_dir):
    z = axis_dir / np.linalg.norm(axis_dir)
    if abs(z[0]) < 0.9:
        arbitrary = np.array([1.0, 0.0, 0.0])
    else:
        arbitrary = np.array([0.0, 1.0, 0.0])
    x = np.cross(arbitrary, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def rotation_matrix_to_rpy_zyx(R):
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
    sy = math.sqrt(r11 * r11 + r21 * r21)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(r32, r33)
        pitch = math.atan2(-r31, sy)
        yaw = math.atan2(r21, r11)
    else:
        roll = math.atan2(-r23, r22)
        pitch = math.atan2(-r31, sy)
        yaw = 0.0
    return roll, pitch, yaw


def detect_holes(mesh, dbscan_eps=0.002, dbscan_min_points=30, diameter_tolerance=0.00075, debug=False):
    """
    radius_tolerance in meters (0.00075 = 0.75 mm)
    """
    centroids, normals = get_triangle_centroids_and_normals(mesh)
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    bbox_min = mesh.get_min_bound()
    bbox_max = mesh.get_max_bound()
    edge_margin = 0.001  # 1 mm to drop perimeter walls
    vertical_normals = np.abs(normals[:, 2]) < 0.2
    away_from_edges = (
        (centroids[:, 0] > bbox_min[0] + edge_margin) &
        (centroids[:, 0] < bbox_max[0] - edge_margin) &
        (centroids[:, 1] > bbox_min[1] + edge_margin) &
        (centroids[:, 1] < bbox_max[1] - edge_margin)
    )
    mask = vertical_normals & away_from_edges
    filtered_centroids = centroids[mask]
    filtered_indices = np.nonzero(mask)[0]
    if filtered_centroids.size == 0:
        return []
    labels = cluster_points_dbscan(filtered_centroids, eps=dbscan_eps, min_points=dbscan_min_points)
    unique_labels = sorted(l for l in set(labels) if l >= 0)
    holes = []
    for lbl in unique_labels:
        cluster_mask = labels == lbl
        if cluster_mask.sum() < 3:
            continue
        triangle_indices = filtered_indices[cluster_mask]
        cluster_triangles = tris[triangle_indices]
        cluster_vertex_ids = np.unique(cluster_triangles.reshape(-1))
        cluster_points = verts[cluster_vertex_ids]
        if cluster_points.shape[0] < 6:
            continue
        try:
            radius, length, entry, exit_, axis_dir = compute_cylinder_params_from_points(cluster_points)
        except ValueError:
            continue
        if axis_dir[2] < 0:
            axis_dir = -axis_dir
            entry, exit_ = exit_, entry
        diameter = 2 * radius
        is_5mm = abs(diameter - 0.005) <= diameter_tolerance
        is_10mm = abs(diameter - 0.010) <= diameter_tolerance
        if debug:
            print(
                f"Cluster {lbl}: points={cluster_points.shape[0]}, diameter={diameter*1000:.3f} mm, "
                f"length={length*1000:.3f} mm"
            )
        if not (is_5mm or is_10mm):
            continue
        R = build_rotation_from_axis(axis_dir)
        roll, pitch, yaw = rotation_matrix_to_rpy_zyx(R)
        holes.append({
            "diameter_m": diameter,
            "length_m": length,
            "origin_xyz": exit_.tolist(),
            "rpy": (roll, pitch, yaw),
            "axis_dir": axis_dir.tolist(),
        })
    return holes


def write_holes_to_csv(holes, out_path):
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["diameter_m", "length_m", "x", "y", "z", "roll", "pitch", "yaw", "axis_x", "axis_y", "axis_z"])
        for h in holes:
            x, y, z = h["origin_xyz"]
            roll, pitch, yaw = h["rpy"]
            ax, ay, az = h["axis_dir"]
            writer.writerow([h["diameter_m"], h["length_m"], x, y, z, roll, pitch, yaw, ax, ay, az])


def main():
    parser = argparse.ArgumentParser(description="Extract 5 mm and 10 mm holes (in meters) from STL mesh")
    parser.add_argument("mesh_path", help="Path to STL mesh (meters)")
    parser.add_argument("--out", "-o", help="Output CSV file", default=None)
    parser.add_argument("--dbscan-eps", type=float, default=0.002, help="DBSCAN epsilon in meters")
    parser.add_argument("--dbscan-min-points", type=int, default=30, help="Minimum points per DBSCAN cluster")
    parser.add_argument(
        "--radius-tol",
        type=float,
        default=0.00075,
        help="Diameter tolerance in meters when matching target holes",
    )
    parser.add_argument("--debug", action="store_true", help="Print debugging info about clusters")
    args = parser.parse_args()

    mesh = load_mesh(args.mesh_path)
    print(f"Loaded mesh with {len(mesh.triangles)} triangles")

    holes = detect_holes(
        mesh,
        dbscan_eps=args.dbscan_eps,
        dbscan_min_points=args.dbscan_min_points,
        diameter_tolerance=args.radius_tol,
        debug=args.debug,
    )
    if not holes:
        print("No 5 mm or 10 mm diameter holes found.")
        return

    for i, h in enumerate(holes):
        x, y, z = h["origin_xyz"]
        print(f"Hole {i}: diam={h['diameter_m']*1000:.2f} mm, len={h['length_m']*1000:.2f} mm, "
              f"xyz=({x:.4f}, {y:.4f}, {z:.4f}) m, rpy={tuple(round(r,4) for r in h['rpy'])}")

    if args.out:
        write_holes_to_csv(holes, args.out)
        print(f"Wrote results to {args.out}")


if __name__ == "__main__":
    main()
