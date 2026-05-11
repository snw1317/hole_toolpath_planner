#include "hole_toolpath_planner/hole_detector.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "hole_detection_common.hpp"

namespace hole_toolpath_planner
{
using detail::BoundaryEdge;
using detail::CircleFitResult;
using detail::CylinderFitSimple;
using detail::EdgeInfo;
using detail::Frame;
using detail::edge_key;
using detail::fit_circle_pratt;
using detail::fit_circle_ransac;
using detail::fit_cylinder_from_points;
using detail::make_frame;
using detail::normalize_or_default;
using detail::polygon_area_2d;
using detail::quaternion_from_frame;
using detail::select_seed;

namespace
{
constexpr size_t kCoverageBins = 36;
constexpr double kDiameterTolerance = 0.00075;

struct RimRefinementOutcome
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double coverage{1.0};
  double rmse{0.0};
  double center_shift{0.0};
  size_t support_count{0};
  bool low_confidence{false};
  bool low_coverage{false};
  bool large_shift{false};
  bool high_rmse{false};
  bool insufficient_support{false};
  bool used_refined_center{false};
};

double angular_coverage_ratio(
  const std::vector<Eigen::Vector2d> & points,
  const Eigen::Vector2d & center,
  const double radius,
  const double inlier_tol)
{
  if (points.empty()) {
    return 0.0;
  }

  std::bitset<kCoverageBins> bins;
  for (const auto & p : points) {
    const Eigen::Vector2d delta = p - center;
    const double dist = delta.norm();
    if (radius > 1e-9 && std::abs(dist - radius) > inlier_tol) {
      continue;
    }

    double angle = std::atan2(delta.y(), delta.x());
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    size_t bin = static_cast<size_t>(std::floor((angle / (2.0 * M_PI)) * static_cast<double>(kCoverageBins)));
    if (bin >= kCoverageBins) {
      bin = kCoverageBins - 1;
    }
    bins.set(bin);
  }

  return static_cast<double>(bins.count()) / static_cast<double>(kCoverageBins);
}

size_t circle_support_count(
  const std::vector<Eigen::Vector2d> & points,
  const Eigen::Vector2d & center,
  const double radius,
  const double inlier_tol)
{
  size_t support = 0;
  for (const auto & p : points) {
    const double dist = (p - center).norm();
    if (std::abs(dist - radius) <= inlier_tol) {
      ++support;
    }
  }
  return support;
}

RimRefinementOutcome refine_rim_center(
  const std::vector<Eigen::Vector3d> & points3d,
  const Eigen::Vector3d & seed_center,
  const Eigen::Vector3d & axis,
  const double initial_radius,
  const srv::DetectHoles::Request & request,
  const PlannerParameters & params)
{
  RimRefinementOutcome out;
  out.center = seed_center;
  out.radius = initial_radius;

  if (!params.surface_circle.refine_enabled) {
    return out;
  }

  const Eigen::Vector3d axis_unit = normalize_or_default(axis, Eigen::Vector3d::UnitZ());
  const double slab = std::max(
    std::max(0.0, params.pose.neighbor_slab_thickness),
    std::max(0.0, params.surface_circle.refine_slab_radius_ratio) * std::max(initial_radius, 1e-6));

  std::vector<Eigen::Vector3d> ring_points;
  ring_points.reserve(points3d.size());
  for (const auto & p : points3d) {
    const double axial_dist = std::abs((p - seed_center).dot(axis_unit));
    if (axial_dist <= slab) {
      ring_points.push_back(p);
    }
  }

  if (ring_points.size() < 3) {
    out.coverage = 0.0;
    out.rmse = std::numeric_limits<double>::infinity();
    out.support_count = ring_points.size();
    out.insufficient_support = true;
    out.low_confidence = true;
    return out;
  }

  const Frame frame = make_frame(axis_unit, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
  std::vector<Eigen::Vector2d> ring_2d;
  ring_2d.reserve(ring_points.size());
  for (const auto & p : ring_points) {
    const Eigen::Vector3d diff = p - seed_center;
    ring_2d.emplace_back(frame.x.dot(diff), frame.y.dot(diff));
  }

  const double inlier_tol = std::max(
    params.surface_circle.circularity_rmse_thresh * 2.0,
    params.cylinder_fit.distance_threshold * 2.0);
  const size_t min_inliers = std::max(
    static_cast<size_t>(params.surface_circle.min_loop_vertices),
    static_cast<size_t>(6));
  const size_t iterations = static_cast<size_t>(
    std::max(10, params.surface_circle.refine_ransac_iterations));

  const CircleFitResult circle = fit_circle_ransac(
    ring_2d,
    iterations,
    inlier_tol,
    min_inliers,
    static_cast<uint32_t>(params.sampling.seed));

  if (!circle.success || !std::isfinite(circle.radius) || circle.radius <= 0.0) {
    out.coverage = angular_coverage_ratio(ring_2d, Eigen::Vector2d::Zero(), std::max(initial_radius, 1e-6), inlier_tol);
    out.rmse = std::numeric_limits<double>::infinity();
    out.support_count = ring_2d.size();
    out.high_rmse = true;
    out.insufficient_support = ring_2d.size() < static_cast<size_t>(params.surface_circle.min_loop_vertices);
    out.low_coverage = out.coverage < params.surface_circle.refine_min_coverage;
    out.low_confidence = out.low_coverage || out.high_rmse || out.insufficient_support;
    return out;
  }

  const Eigen::Vector3d refined_center = seed_center + frame.x * circle.center.x() + frame.y * circle.center.y();
  const double refined_diameter = 2.0 * circle.radius;
  const double min_diameter = request.min_diameter > 0.0 ? request.min_diameter : 0.0;
  const double max_diameter = request.max_diameter > 0.0 ?
    request.max_diameter : std::numeric_limits<double>::max();

  out.coverage = angular_coverage_ratio(ring_2d, circle.center, circle.radius, inlier_tol);
  out.rmse = circle.rmse;
  out.center_shift = (refined_center - seed_center).norm();
  out.support_count = circle_support_count(ring_2d, circle.center, circle.radius, inlier_tol);
  out.low_coverage = out.coverage < params.surface_circle.refine_min_coverage;
  out.insufficient_support = out.support_count < static_cast<size_t>(params.surface_circle.min_loop_vertices);
  out.high_rmse = !std::isfinite(circle.rmse) || circle.rmse > params.surface_circle.circularity_rmse_thresh;
  out.large_shift =
    out.center_shift > params.surface_circle.refine_shift_max_ratio * std::max(circle.radius, 1e-6);

  const bool diameter_ok =
    refined_diameter + kDiameterTolerance >= min_diameter &&
    refined_diameter - kDiameterTolerance <= max_diameter;

  const bool accept_refined =
    diameter_ok &&
    !out.high_rmse &&
    !out.large_shift;

  if (accept_refined) {
    out.center = refined_center;
    out.radius = circle.radius;
    out.used_refined_center = true;
  }

  out.low_confidence = out.low_coverage || out.insufficient_support || out.high_rmse || out.large_shift;
  return out;
}

HoleDetector::HoleQuality to_hole_quality(const RimRefinementOutcome & refinement)
{
  HoleDetector::HoleQuality quality;
  quality.low_confidence = refinement.low_confidence;
  quality.coverage = static_cast<float>(refinement.coverage);
  quality.rmse = std::isfinite(refinement.rmse) ? static_cast<float>(refinement.rmse) : std::numeric_limits<float>::infinity();
  quality.center_shift = static_cast<float>(refinement.center_shift);
  return quality;
}
}  // namespace

std::vector<msg::Hole> HoleDetector::detect_surface_clusters(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SurfaceDiagnostics * diagnostics,
  std::vector<HoleQuality> * qualities) const
{
  if (diagnostics) {
    diagnostics->attempted = true;
    diagnostics->face_count = mesh.polygons.size();
    diagnostics->boundary_edge_count = 0;
    diagnostics->loops_total = 0;
    diagnostics->loops_considered = 0;
    diagnostics->circle_fit_success = 0;
    diagnostics->detections_emitted = 0;
    diagnostics->mesh_empty = mesh.polygons.empty();
    diagnostics->vertices_empty = false;
    diagnostics->boundary_edges_empty = false;
    diagnostics->loops_invalid_topology = 0;
    diagnostics->loops_too_small = 0;
    diagnostics->loops_invalid_vertices = 0;
    diagnostics->loops_outer_skipped = 0;
    diagnostics->circle_fit_failures = 0;
    diagnostics->rmse_rejections = 0;
    diagnostics->radius_rejections = 0;
    diagnostics->low_confidence_count = 0;
    diagnostics->low_coverage_count = 0;
    diagnostics->large_shift_count = 0;
  }

  std::vector<msg::Hole> detections;

  if (mesh.polygons.empty()) {
    if (diagnostics) {
      diagnostics->mesh_empty = true;
    }
    RCLCPP_WARN(logger_, "Surface cluster mode cannot run: mesh contains no polygons.");
    return detections;
  }

  pcl::PointCloud<pcl::PointXYZ> vertex_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_cloud);
  if (vertex_cloud.empty()) {
    if (diagnostics) {
      diagnostics->vertices_empty = true;
    }
    RCLCPP_WARN(logger_, "Surface cluster mode cannot run: mesh has no vertices.");
    return detections;
  }

  std::vector<Eigen::Vector3d> vertices(vertex_cloud.size());
  for (size_t i = 0; i < vertex_cloud.size(); ++i) {
    vertices[i] = Eigen::Vector3d{vertex_cloud[i].x, vertex_cloud[i].y, vertex_cloud[i].z};
  }

  Eigen::Vector3d bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d bbox_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto & v : vertices) {
    bbox_min = bbox_min.cwiseMin(v);
    bbox_max = bbox_max.cwiseMax(v);
  }

  constexpr double normal_z_threshold = 0.2;
  constexpr double edge_margin = 0.001;
  const double cluster_tolerance = std::max(1e-6, params_.detection.cluster_tolerance);
  const int min_cluster_size = std::max(10, params_.surface_circle.min_loop_vertices);
  constexpr double diameter_tolerance = 0.00075;
  constexpr double length_tolerance = 1e-5;

  std::vector<Eigen::Vector3d> centroids;
  centroids.reserve(mesh.polygons.size());
  std::vector<size_t> centroid_triangle_indices;
  centroid_triangle_indices.reserve(mesh.polygons.size());

  bool warned_non_tri = false;
  for (size_t face_idx = 0; face_idx < mesh.polygons.size(); ++face_idx) {
    const auto & poly = mesh.polygons[face_idx];
    if (poly.vertices.size() != 3) {
      if (!warned_non_tri) {
        RCLCPP_WARN(
          logger_,
          "Surface cluster mode currently supports triangle meshes only; ignoring non-triangles.");
        warned_non_tri = true;
      }
      continue;
    }

    std::array<uint32_t, 3> vids{};
    bool indices_valid = true;
    for (size_t i = 0; i < 3; ++i) {
      vids[i] = poly.vertices[i];
      if (vids[i] >= vertices.size()) {
        indices_valid = false;
        break;
      }
    }
    if (!indices_valid) {
      continue;
    }

    const Eigen::Vector3d & v0 = vertices[vids[0]];
    const Eigen::Vector3d & v1 = vertices[vids[1]];
    const Eigen::Vector3d & v2 = vertices[vids[2]];

    const Eigen::Vector3d e0 = v1 - v0;
    const Eigen::Vector3d e1 = v2 - v0;
    Eigen::Vector3d normal = e0.cross(e1);
    const double normal_norm = normal.norm();
    if (normal_norm < 1e-12) {
      continue;
    }
    normal /= normal_norm;

    const Eigen::Vector3d centroid = (v0 + v1 + v2) / 3.0;

    if (std::abs(normal.z()) >= normal_z_threshold) {
      continue;
    }

    const bool inside_x =
      centroid.x() > bbox_min.x() + edge_margin && centroid.x() < bbox_max.x() - edge_margin;
    const bool inside_y =
      centroid.y() > bbox_min.y() + edge_margin && centroid.y() < bbox_max.y() - edge_margin;

    if (!inside_x || !inside_y) {
      continue;
    }

    centroids.push_back(centroid);
    centroid_triangle_indices.push_back(face_idx);
  }

  if (diagnostics) {
    diagnostics->boundary_edge_count = centroids.size();
  }

  if (centroids.empty()) {
    if (diagnostics) {
      diagnostics->boundary_edges_empty = true;
    }
    RCLCPP_DEBUG(logger_, "Surface cluster mode: no candidate cylindrical wall faces found.");
    return detections;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  centroid_cloud->points.reserve(centroids.size());
  for (const auto & c : centroids) {
    centroid_cloud->points.emplace_back(
      static_cast<float>(c.x()),
      static_cast<float>(c.y()),
      static_cast<float>(c.z()));
  }
  centroid_cloud->width = centroid_cloud->points.size();
  centroid_cloud->height = 1;

  if (centroid_cloud->points.size() < static_cast<size_t>(min_cluster_size)) {
    return detections;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  search_tree->setInputCloud(centroid_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
  extractor.setInputCloud(centroid_cloud);
  extractor.setSearchMethod(search_tree);
  extractor.setClusterTolerance(cluster_tolerance);
  extractor.setMinClusterSize(min_cluster_size);
  extractor.setMaxClusterSize(static_cast<int>(centroid_cloud->points.size()));

  std::vector<pcl::PointIndices> clusters;
  extractor.extract(clusters);

  if (diagnostics) {
    diagnostics->loops_total = clusters.size();
    diagnostics->loops_considered = clusters.size();
  }

  const double min_diameter = request.min_diameter > 0.0 ? request.min_diameter : 0.0;
  const double max_diameter = request.max_diameter > 0.0 ?
    request.max_diameter : std::numeric_limits<double>::max();
  const double min_length = request.min_length > 0.0 ? request.min_length : 0.0;

  for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
    const auto & indices = clusters[cluster_idx].indices;
    if (indices.size() < static_cast<size_t>(min_cluster_size)) {
      continue;
    }

    std::unordered_set<uint32_t> vertex_ids;
    vertex_ids.reserve(indices.size() * 3);

    for (const int centroid_idx : indices) {
      if (centroid_idx < 0 || static_cast<size_t>(centroid_idx) >= centroid_triangle_indices.size()) {
        continue;
      }
      const size_t tri_idx = centroid_triangle_indices[static_cast<size_t>(centroid_idx)];
      const auto & poly = mesh.polygons[tri_idx];
      for (const uint32_t vid : poly.vertices) {
        if (vid < vertices.size()) {
          vertex_ids.insert(vid);
        }
      }
    }

    std::vector<Eigen::Vector3d> cluster_vertices;
    cluster_vertices.reserve(vertex_ids.size());
    for (const uint32_t vid : vertex_ids) {
      cluster_vertices.push_back(vertices[vid]);
    }

    if (cluster_vertices.size() < 6) {
      continue;
    }

    const CylinderFitSimple fit = fit_cylinder_from_points(cluster_vertices);
    if (!fit.valid) {
      continue;
    }

    const double diameter = 2.0 * fit.radius;
    if (diameter + diameter_tolerance < min_diameter) {
      continue;
    }
    if (diameter - diameter_tolerance > max_diameter) {
      continue;
    }
    if (min_length > 0.0 && fit.length + length_tolerance < min_length) {
      continue;
    }

    const RimRefinementOutcome refinement = refine_rim_center(
      cluster_vertices,
      fit.top,
      fit.axis,
      fit.radius,
      request,
      params_);
    const Frame frame = make_frame(fit.axis, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
    const double output_diameter = refinement.used_refined_center ? 2.0 * refinement.radius : diameter;

    msg::Hole hole;
    hole.kind = msg::Hole::SURFACE_CIRCLE;
    hole.diameter = static_cast<float>(output_diameter);
    hole.length = static_cast<float>(fit.length);
    hole.pose.position.x = refinement.center.x();
    hole.pose.position.y = refinement.center.y();
    hole.pose.position.z = refinement.center.z();
    hole.pose.orientation = quaternion_from_frame(frame);
    hole.axis.x = frame.z.x();
    hole.axis.y = frame.z.y();
    hole.axis.z = frame.z.z();

    detections.push_back(std::move(hole));

    if (diagnostics) {
      ++diagnostics->detections_emitted;
      ++diagnostics->circle_fit_success;
      if (refinement.low_confidence) {
        ++diagnostics->low_confidence_count;
      }
      if (refinement.low_coverage) {
        ++diagnostics->low_coverage_count;
      }
      if (refinement.large_shift) {
        ++diagnostics->large_shift_count;
      }
    }

    if (qualities != nullptr) {
      qualities->push_back(to_hole_quality(refinement));
    }

    RCLCPP_DEBUG(
      logger_,
      "Surface cluster %zu: triangles=%zu, vertices=%zu, diameter=%.3f mm, length=%.3f mm",
      cluster_idx,
      indices.size(),
      cluster_vertices.size(),
      diameter * 1000.0,
      fit.length * 1000.0);
  }

  return detections;
}

std::vector<msg::Hole> HoleDetector::detect_surface_clusters_radial(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SurfaceDiagnostics * diagnostics,
  std::vector<HoleQuality> * qualities) const
{
  if (diagnostics) {
    diagnostics->attempted = true;
    diagnostics->face_count = mesh.polygons.size();
    diagnostics->boundary_edge_count = 0;
    diagnostics->loops_total = 0;
    diagnostics->loops_considered = 0;
    diagnostics->circle_fit_success = 0;
    diagnostics->detections_emitted = 0;
    diagnostics->mesh_empty = mesh.polygons.empty();
    diagnostics->vertices_empty = false;
    diagnostics->boundary_edges_empty = false;
    diagnostics->loops_invalid_topology = 0;
    diagnostics->loops_too_small = 0;
    diagnostics->loops_invalid_vertices = 0;
    diagnostics->loops_outer_skipped = 0;
    diagnostics->circle_fit_failures = 0;
    diagnostics->rmse_rejections = 0;
    diagnostics->radius_rejections = 0;
    diagnostics->low_confidence_count = 0;
    diagnostics->low_coverage_count = 0;
    diagnostics->large_shift_count = 0;
  }

  std::vector<msg::Hole> detections;

  if (mesh.polygons.empty()) {
    if (diagnostics) {
      diagnostics->mesh_empty = true;
    }
    RCLCPP_WARN(logger_, "Surface radial cluster mode cannot run: mesh contains no polygons.");
    return detections;
  }

  pcl::PointCloud<pcl::PointXYZ> vertex_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_cloud);
  if (vertex_cloud.empty()) {
    if (diagnostics) {
      diagnostics->vertices_empty = true;
    }
    RCLCPP_WARN(logger_, "Surface radial cluster mode cannot run: mesh has no vertices.");
    return detections;
  }

  std::vector<Eigen::Vector3d> vertices(vertex_cloud.size());
  Eigen::Vector3d mesh_center = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < vertex_cloud.size(); ++i) {
    vertices[i] = Eigen::Vector3d{vertex_cloud[i].x, vertex_cloud[i].y, vertex_cloud[i].z};
    mesh_center += vertices[i];
  }
  mesh_center /= static_cast<double>(vertex_cloud.size());

  const double radial_dot_threshold =
    std::clamp(params_.detection.radial_dot_threshold, 0.0, 1.0);
  const double cluster_tolerance = std::max(1e-6, params_.detection.cluster_tolerance);
  const int min_cluster_size = std::max(10, params_.surface_circle.min_loop_vertices);
  constexpr double diameter_tolerance = 0.00075;
  constexpr double length_tolerance = 1e-5;

  std::vector<Eigen::Vector3d> centroids;
  centroids.reserve(mesh.polygons.size());
  std::vector<size_t> centroid_triangle_indices;
  centroid_triangle_indices.reserve(mesh.polygons.size());

  bool warned_non_tri = false;
  for (size_t face_idx = 0; face_idx < mesh.polygons.size(); ++face_idx) {
    const auto & poly = mesh.polygons[face_idx];
    if (poly.vertices.size() != 3) {
      if (!warned_non_tri) {
        RCLCPP_WARN(
          logger_,
          "Surface radial cluster mode currently supports triangle meshes only; ignoring non-triangles.");
        warned_non_tri = true;
      }
      continue;
    }

    std::array<uint32_t, 3> vids{};
    bool indices_valid = true;
    for (size_t i = 0; i < 3; ++i) {
      vids[i] = poly.vertices[i];
      if (vids[i] >= vertices.size()) {
        indices_valid = false;
        break;
      }
    }
    if (!indices_valid) {
      continue;
    }

    const Eigen::Vector3d & v0 = vertices[vids[0]];
    const Eigen::Vector3d & v1 = vertices[vids[1]];
    const Eigen::Vector3d & v2 = vertices[vids[2]];

    const Eigen::Vector3d e0 = v1 - v0;
    const Eigen::Vector3d e1 = v2 - v0;
    Eigen::Vector3d normal = e0.cross(e1);
    const double normal_norm = normal.norm();
    if (normal_norm < 1e-12) {
      continue;
    }
    normal /= normal_norm;

    const Eigen::Vector3d centroid = (v0 + v1 + v2) / 3.0;
    const Eigen::Vector3d radial = centroid - mesh_center;
    const double radial_norm = radial.norm();
    if (radial_norm < 1e-12) {
      continue;
    }
    const Eigen::Vector3d radial_unit = radial / radial_norm;
    const double radial_dot = std::abs(normal.dot(radial_unit));
    if (radial_dot > radial_dot_threshold) {
      continue;
    }

    centroids.push_back(centroid);
    centroid_triangle_indices.push_back(face_idx);
  }

  if (diagnostics) {
    diagnostics->boundary_edge_count = centroids.size();
  }

  if (centroids.empty()) {
    if (diagnostics) {
      diagnostics->boundary_edges_empty = true;
    }
    RCLCPP_DEBUG(logger_, "Surface radial cluster mode: no candidate cylindrical wall faces found.");
    return detections;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  centroid_cloud->points.reserve(centroids.size());
  for (const auto & c : centroids) {
    centroid_cloud->points.emplace_back(
      static_cast<float>(c.x()),
      static_cast<float>(c.y()),
      static_cast<float>(c.z()));
  }
  centroid_cloud->width = centroid_cloud->points.size();
  centroid_cloud->height = 1;

  if (centroid_cloud->points.size() < static_cast<size_t>(min_cluster_size)) {
    return detections;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  search_tree->setInputCloud(centroid_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
  extractor.setInputCloud(centroid_cloud);
  extractor.setSearchMethod(search_tree);
  extractor.setClusterTolerance(cluster_tolerance);
  extractor.setMinClusterSize(min_cluster_size);
  extractor.setMaxClusterSize(static_cast<int>(centroid_cloud->points.size()));

  std::vector<pcl::PointIndices> clusters;
  extractor.extract(clusters);

  if (diagnostics) {
    diagnostics->loops_total = clusters.size();
    diagnostics->loops_considered = clusters.size();
  }

  const double min_diameter = request.min_diameter > 0.0 ? request.min_diameter : 0.0;
  const double max_diameter = request.max_diameter > 0.0 ?
    request.max_diameter : std::numeric_limits<double>::max();
  const double min_length = request.min_length > 0.0 ? request.min_length : 0.0;

  for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
    const auto & indices = clusters[cluster_idx].indices;
    if (indices.size() < static_cast<size_t>(min_cluster_size)) {
      continue;
    }

    std::unordered_set<uint32_t> vertex_ids;
    vertex_ids.reserve(indices.size() * 3);

    for (const int centroid_idx : indices) {
      if (centroid_idx < 0 || static_cast<size_t>(centroid_idx) >= centroid_triangle_indices.size()) {
        continue;
      }
      const size_t tri_idx = centroid_triangle_indices[static_cast<size_t>(centroid_idx)];
      const auto & poly = mesh.polygons[tri_idx];
      for (const uint32_t vid : poly.vertices) {
        if (vid < vertices.size()) {
          vertex_ids.insert(vid);
        }
      }
    }

    std::vector<Eigen::Vector3d> cluster_vertices;
    cluster_vertices.reserve(vertex_ids.size());
    for (const uint32_t vid : vertex_ids) {
      cluster_vertices.push_back(vertices[vid]);
    }

    if (cluster_vertices.size() < 6) {
      continue;
    }

    const CylinderFitSimple fit = fit_cylinder_from_points(cluster_vertices);
    if (!fit.valid) {
      continue;
    }

    const double diameter = 2.0 * fit.radius;
    if (diameter + diameter_tolerance < min_diameter) {
      continue;
    }
    if (diameter - diameter_tolerance > max_diameter) {
      continue;
    }
    if (min_length > 0.0 && fit.length + length_tolerance < min_length) {
      continue;
    }

    msg::Hole hole;
    hole.kind = msg::Hole::SURFACE_CIRCLE;
    const double top_dist = (fit.top - mesh_center).squaredNorm();
    const double bottom_dist = (fit.bottom - mesh_center).squaredNorm();
    const Eigen::Vector3d entry = (top_dist >= bottom_dist) ? fit.top : fit.bottom;

    Eigen::Vector3d radial_out = entry - mesh_center;
    radial_out = normalize_or_default(radial_out, fit.axis);
    const RimRefinementOutcome refinement = refine_rim_center(
      cluster_vertices,
      entry,
      fit.axis,
      fit.radius,
      request,
      params_);

    const double output_diameter = refinement.used_refined_center ? 2.0 * refinement.radius : diameter;
    hole.diameter = static_cast<float>(output_diameter);
    hole.length = static_cast<float>(fit.length);

    Eigen::Vector3d entry_center = refinement.center;
    Eigen::Vector3d z_dir = fit.axis;

    const double z_dot = z_dir.dot(radial_out);
    if (std::abs(z_dot) < 0.3) {
      z_dir = radial_out;
    } else if (z_dot < 0.0) {
      z_dir = -z_dir;
    }

    const Frame frame = make_frame(z_dir, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());

    hole.pose.position.x = entry_center.x();
    hole.pose.position.y = entry_center.y();
    hole.pose.position.z = entry_center.z();
    hole.pose.orientation = quaternion_from_frame(frame);
    hole.axis.x = frame.z.x();
    hole.axis.y = frame.z.y();
    hole.axis.z = frame.z.z();

    detections.push_back(std::move(hole));

    if (diagnostics) {
      ++diagnostics->detections_emitted;
      ++diagnostics->circle_fit_success;
      if (refinement.low_confidence) {
        ++diagnostics->low_confidence_count;
      }
      if (refinement.low_coverage) {
        ++diagnostics->low_coverage_count;
      }
      if (refinement.large_shift) {
        ++diagnostics->large_shift_count;
      }
    }
    if (qualities != nullptr) {
      qualities->push_back(to_hole_quality(refinement));
    }
  }

  return detections;
}

std::vector<msg::Hole> HoleDetector::detect_surface_mode_legacy(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SurfaceDiagnostics * diagnostics,
  std::vector<HoleQuality> * qualities) const
{
  if (diagnostics) {
    diagnostics->attempted = true;
    diagnostics->face_count = mesh.polygons.size();
    diagnostics->boundary_edge_count = 0;
    diagnostics->loops_total = 0;
    diagnostics->loops_considered = 0;
    diagnostics->circle_fit_success = 0;
    diagnostics->detections_emitted = 0;
    diagnostics->mesh_empty = mesh.polygons.empty();
    diagnostics->vertices_empty = false;
    diagnostics->boundary_edges_empty = false;
    diagnostics->loops_invalid_topology = 0;
    diagnostics->loops_too_small = 0;
    diagnostics->loops_invalid_vertices = 0;
    diagnostics->loops_outer_skipped = 0;
    diagnostics->circle_fit_failures = 0;
    diagnostics->rmse_rejections = 0;
    diagnostics->radius_rejections = 0;
    diagnostics->low_confidence_count = 0;
    diagnostics->low_coverage_count = 0;
    diagnostics->large_shift_count = 0;
  }

  std::vector<msg::Hole> detections;

  if (mesh.polygons.empty()) {
    if (diagnostics) {
      diagnostics->mesh_empty = true;
    }
    RCLCPP_WARN_ONCE(logger_, "Surface-mode requested but mesh contains no polygons.");
    return detections;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  if (cloud.empty()) {
    if (diagnostics) {
      diagnostics->vertices_empty = true;
    }
    RCLCPP_WARN(logger_, "Surface-mode cannot run: mesh has no vertices.");
    return detections;
  }

  const size_t vertex_count = cloud.size();
  const size_t face_count = mesh.polygons.size();

  std::vector<Eigen::Vector3d> vertices(vertex_count);
  for (size_t i = 0; i < vertex_count; ++i) {
    vertices[i] = Eigen::Vector3d{cloud[i].x, cloud[i].y, cloud[i].z};
  }

  std::unordered_map<uint64_t, EdgeInfo> edges;
  edges.reserve(face_count * 3);

  std::vector<bool> face_valid(face_count, false);
  std::vector<Eigen::Vector3d> face_normals(face_count, Eigen::Vector3d::Zero());
  std::vector<double> face_areas(face_count, 0.0);

  bool warned_non_tri = false;
  for (size_t face_idx = 0; face_idx < face_count; ++face_idx) {
    const auto & poly = mesh.polygons[face_idx];
    if (poly.vertices.size() != 3) {
      if (!warned_non_tri) {
        RCLCPP_WARN(logger_, "Surface-mode currently supports triangle meshes only; ignoring non-triangles.");
        warned_non_tri = true;
      }
      continue;
    }

    std::array<uint32_t, 3> vids{};
    bool indices_valid = true;
    for (size_t i = 0; i < 3; ++i) {
      vids[i] = poly.vertices[i];
      if (vids[i] >= vertex_count) {
        indices_valid = false;
        break;
      }
    }
    if (!indices_valid) {
      RCLCPP_WARN(logger_, "Skipping face %zu with out-of-range vertex index", face_idx);
      continue;
    }

    const Eigen::Vector3d & v0 = vertices[vids[0]];
    const Eigen::Vector3d & v1 = vertices[vids[1]];
    const Eigen::Vector3d & v2 = vertices[vids[2]];

    const Eigen::Vector3d e0 = v1 - v0;
    const Eigen::Vector3d e1 = v2 - v0;
    Eigen::Vector3d normal = e0.cross(e1);
    const double area2 = normal.norm();
    if (area2 < 1e-12) {
      continue;  // Degenerate triangle
    }

    face_valid[face_idx] = true;
    normal /= area2;
    const double area = 0.5 * area2;
    face_normals[face_idx] = normal;
    face_areas[face_idx] = area;

    const std::array<std::pair<uint32_t, uint32_t>, 3> oriented_edges{
      std::make_pair(vids[0], vids[1]),
      std::make_pair(vids[1], vids[2]),
      std::make_pair(vids[2], vids[0])};

    for (const auto & edge : oriented_edges) {
      EdgeInfo & info = edges[edge_key(edge.first, edge.second)];
      if (info.first_face == std::numeric_limits<uint32_t>::max()) {
        info.first_face = static_cast<uint32_t>(face_idx);
        info.first_oriented = edge;
      } else if (
        info.second_face == std::numeric_limits<uint32_t>::max() &&
        info.first_face != static_cast<uint32_t>(face_idx))
      {
        info.second_face = static_cast<uint32_t>(face_idx);
        info.second_oriented = edge;
      }
    }
  }

  std::vector<std::vector<uint32_t>> adjacency(face_count);
  for (const auto & kv : edges) {
    const auto & info = kv.second;
    if (
      info.first_face != std::numeric_limits<uint32_t>::max() &&
      info.second_face != std::numeric_limits<uint32_t>::max() &&
      face_valid[info.first_face] && face_valid[info.second_face])
    {
      adjacency[info.first_face].push_back(info.second_face);
      adjacency[info.second_face].push_back(info.first_face);
    }
  }

  std::vector<int> component(face_count, -1);
  std::vector<Eigen::Vector3d> component_normals;
  component_normals.reserve(8);

  for (size_t face_idx = 0; face_idx < face_count; ++face_idx) {
    if (!face_valid[face_idx] || component[face_idx] != -1) {
      continue;
    }

    const int comp_id = static_cast<int>(component_normals.size());
    component_normals.emplace_back(Eigen::Vector3d::Zero());

    std::queue<uint32_t> queue;
    queue.push(static_cast<uint32_t>(face_idx));

    while (!queue.empty()) {
      const uint32_t current = queue.front();
      queue.pop();

      if (!face_valid[current] || component[current] != -1) {
        continue;
      }

      component[current] = comp_id;
      component_normals[comp_id] += face_normals[current] * face_areas[current];

      for (const uint32_t neighbor : adjacency[current]) {
        if (neighbor < face_count && face_valid[neighbor] && component[neighbor] == -1) {
          queue.push(neighbor);
        }
      }
    }
  }

  std::vector<Eigen::Vector3d> component_outward(component_normals.size(), Eigen::Vector3d::Zero());
  for (size_t i = 0; i < component_normals.size(); ++i) {
    component_outward[i] = normalize_or_default(component_normals[i], Eigen::Vector3d::Zero());
  }

  std::vector<BoundaryEdge> boundary_edges;
  boundary_edges.reserve(edges.size());
  std::unordered_map<int, std::unordered_map<uint32_t, std::vector<size_t>>> edges_by_component;

  for (const auto & kv : edges) {
    const EdgeInfo & info = kv.second;
    if (info.first_face == std::numeric_limits<uint32_t>::max()) {
      continue;
    }
    if (!face_valid[info.first_face]) {
      continue;
    }

    const bool has_pair =
      info.second_face != std::numeric_limits<uint32_t>::max() &&
      face_valid[info.second_face];
    if (has_pair) {
      continue;
    }

    const int comp_id = component[info.first_face];
    if (comp_id < 0) {
      continue;
    }

    const auto start = info.first_oriented.first;
    const auto end = info.first_oriented.second;
    const size_t index = boundary_edges.size();
    boundary_edges.push_back(BoundaryEdge{start, end, info.first_face, comp_id, false});
    edges_by_component[comp_id][start].push_back(index);
    if (diagnostics) {
      ++diagnostics->boundary_edge_count;
    }
  }

  if (boundary_edges.empty()) {
    if (diagnostics) {
      diagnostics->boundary_edges_empty = true;
    }
    return detections;
  }

  struct LoopInfo
  {
    int component;
    std::vector<uint32_t> vertices;
  };

  std::vector<LoopInfo> loops;
  loops.reserve(boundary_edges.size());

  for (size_t i = 0; i < boundary_edges.size(); ++i) {
    if (boundary_edges[i].visited) {
      continue;
    }

    const int comp_id = boundary_edges[i].component;
    auto comp_it = edges_by_component.find(comp_id);
    if (comp_it == edges_by_component.end()) {
      continue;
    }

    std::vector<uint32_t> loop_vertices;
    loop_vertices.reserve(16);

    uint32_t start_vertex = boundary_edges[i].start;
    uint32_t current_vertex = start_vertex;
    size_t current_edge = i;
    bool valid_loop = true;
    size_t iterations = 0;

    while (true) {
      if (current_edge >= boundary_edges.size()) {
        valid_loop = false;
        break;
      }

      auto & edge = boundary_edges[current_edge];
      if (edge.visited) {
        valid_loop = false;
        break;
      }

      edge.visited = true;
      loop_vertices.push_back(edge.start);
      current_vertex = edge.end;

      if (current_vertex == start_vertex) {
        break;
      }

      auto vertex_it = comp_it->second.find(current_vertex);
      if (vertex_it == comp_it->second.end()) {
        valid_loop = false;
        break;
      }

      size_t next_edge_index = std::numeric_limits<size_t>::max();
      for (const size_t candidate : vertex_it->second) {
        if (!boundary_edges[candidate].visited) {
          next_edge_index = candidate;
          break;
        }
      }

      if (next_edge_index == std::numeric_limits<size_t>::max()) {
        valid_loop = false;
        break;
      }

      current_edge = next_edge_index;
      ++iterations;
      if (iterations > boundary_edges.size() * 2ULL) {
        valid_loop = false;
        break;
      }
    }

    if (!valid_loop) {
      if (diagnostics) {
        ++diagnostics->loops_invalid_topology;
      }
      continue;
    }

    if (loop_vertices.size() < static_cast<size_t>(params_.surface_circle.min_loop_vertices)) {
      if (diagnostics) {
        ++diagnostics->loops_too_small;
      }
      continue;
    }

    loops.push_back(LoopInfo{comp_id, std::move(loop_vertices)});
  }

  if (loops.empty()) {
    return detections;
  }

  struct LoopMetrics
  {
    int component;
    std::vector<uint32_t> vertex_ids;
    std::vector<Eigen::Vector2d> projected;
    Eigen::Vector3d centroid;
    Eigen::Vector3d normal;
    Eigen::Vector3d axis_major;
    Eigen::Vector3d axis_minor;
    CircleFitResult circle;
    double area;
  };

  std::vector<LoopMetrics> metrics;
  metrics.reserve(loops.size());

  for (const auto & loop : loops) {
    std::vector<Eigen::Vector3d> points3d;
    points3d.reserve(loop.vertices.size());
    bool has_invalid = false;
    for (const uint32_t vid : loop.vertices) {
      if (vid >= vertex_count) {
        if (diagnostics) {
          ++diagnostics->loops_invalid_vertices;
        }
        has_invalid = true;
        break;
      }
      points3d.push_back(vertices[vid]);
    }
    if (has_invalid || points3d.size() < 3) {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : points3d) {
      centroid += p;
    }
    centroid /= static_cast<double>(points3d.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & p : points3d) {
      const Eigen::Vector3d diff = p - centroid;
      covariance += diff * diff.transpose();
    }
    covariance /= static_cast<double>(points3d.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }

    Eigen::Vector3d normal = normalize_or_default(solver.eigenvectors().col(0), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d axis_major = normalize_or_default(solver.eigenvectors().col(2), Eigen::Vector3d::UnitX());
    Eigen::Vector3d axis_minor = normalize_or_default(solver.eigenvectors().col(1), Eigen::Vector3d::UnitY());
    if (axis_major.cross(axis_minor).dot(normal) < 0.0) {
      axis_minor = -axis_minor;
    }

    std::vector<Eigen::Vector2d> projected;
    projected.reserve(points3d.size());
    for (const auto & p : points3d) {
      const Eigen::Vector3d diff = p - centroid;
      projected.emplace_back(axis_major.dot(diff), axis_minor.dot(diff));
    }

    const double area = std::abs(polygon_area_2d(projected));

    LoopMetrics metric{
      loop.component,
      loop.vertices,
      std::move(projected),
      centroid,
      normal,
      axis_major,
      axis_minor,
      fit_circle_pratt(projected),
      area};
    metrics.push_back(std::move(metric));
  }

  if (diagnostics) {
    diagnostics->loops_total = metrics.size();
  }

  if (metrics.empty()) {
    return detections;
  }

  std::unordered_map<int, size_t> outer_loop_index;
  for (size_t i = 0; i < metrics.size(); ++i) {
    const int comp_id = metrics[i].component;
    const double area = metrics[i].area;
    auto it = outer_loop_index.find(comp_id);
    if (it == outer_loop_index.end() || area > metrics[it->second].area) {
      outer_loop_index[comp_id] = i;
    }
  }

  const double request_min_radius = 0.5 * std::max(0.0, static_cast<double>(request.min_diameter));
  double request_max_radius = 0.5 * std::max(0.0, static_cast<double>(request.max_diameter));
  if (request_max_radius > 0.0 && request_max_radius < request_min_radius) {
    RCLCPP_WARN(
      logger_,
      "Request max_diameter (%.4f m) is less than min_diameter (%.4f m); clamping to the minimum.",
      static_cast<double>(request.max_diameter),
      static_cast<double>(request.min_diameter));
    request_max_radius = request_min_radius;
  }
  const double min_radius_threshold = std::max(params_.surface_circle.min_radius, request_min_radius);
  const double max_radius_request = request_max_radius > 0.0 ? request_max_radius : std::numeric_limits<double>::infinity();
  const double max_radius_threshold = std::min(params_.surface_circle.max_radius, max_radius_request);
  const Eigen::Vector3d hint = normalize_or_default(
    Eigen::Vector3d{params_.surface_circle.into_hint[0], params_.surface_circle.into_hint[1], params_.surface_circle.into_hint[2]},
    Eigen::Vector3d::UnitZ());

  for (size_t i = 0; i < metrics.size(); ++i) {
    if (outer_loop_index.count(metrics[i].component) && outer_loop_index[metrics[i].component] == i) {
      if (diagnostics) {
        ++diagnostics->loops_outer_skipped;
      }
      continue;  // Skip outer boundary
    }

    const auto & metric = metrics[i];
    if (diagnostics) {
      ++diagnostics->loops_considered;
    }
    if (!metric.circle.success) {
      if (diagnostics) {
        ++diagnostics->circle_fit_failures;
      }
      continue;
    }
    if (diagnostics) {
      ++diagnostics->circle_fit_success;
    }
    if (metric.circle.rmse > params_.surface_circle.circularity_rmse_thresh) {
      if (diagnostics) {
        ++diagnostics->rmse_rejections;
      }
      continue;
    }

    const double radius = metric.circle.radius;
    if (radius < min_radius_threshold || radius > max_radius_threshold) {
      if (diagnostics) {
        ++diagnostics->radius_rejections;
      }
      continue;
    }
    const double diameter = 2.0 * radius;

    Eigen::Vector3d z_dir = normalize_or_default(metric.normal, Eigen::Vector3d::UnitZ());
    const int comp_id = metric.component;
    if (comp_id >= 0 && static_cast<size_t>(comp_id) < component_outward.size()) {
      const Eigen::Vector3d outward = component_outward[comp_id];
      if (outward.norm() > 1e-6) {
        if (z_dir.dot(outward) > 0.0) {
          z_dir = -z_dir;
        }
      } else if (z_dir.dot(hint) < 0.0) {
        z_dir = -z_dir;
      }
    } else if (z_dir.dot(hint) < 0.0) {
      z_dir = -z_dir;
    }

    const Eigen::Vector3d base_center3d = metric.centroid +
      metric.axis_major * metric.circle.center.x() +
      metric.axis_minor * metric.circle.center.y();
    const double inlier_tol = std::max(
      params_.surface_circle.circularity_rmse_thresh * 2.0,
      params_.cylinder_fit.distance_threshold * 2.0);

    RimRefinementOutcome refinement;
    refinement.center = base_center3d;
    refinement.radius = radius;
    refinement.coverage = angular_coverage_ratio(metric.projected, metric.circle.center, radius, inlier_tol);
    refinement.rmse = metric.circle.rmse;
    refinement.center_shift = 0.0;
    refinement.support_count = circle_support_count(metric.projected, metric.circle.center, radius, inlier_tol);
    refinement.low_coverage = refinement.coverage < params_.surface_circle.refine_min_coverage;
    refinement.insufficient_support =
      refinement.support_count < static_cast<size_t>(params_.surface_circle.min_loop_vertices);
    refinement.high_rmse =
      !std::isfinite(refinement.rmse) || refinement.rmse > params_.surface_circle.circularity_rmse_thresh;
    refinement.large_shift = false;
    refinement.low_confidence =
      refinement.low_coverage || refinement.insufficient_support || refinement.high_rmse;

    const bool should_refine =
      params_.surface_circle.refine_enabled &&
      (refinement.low_coverage ||
      metric.circle.rmse > 0.5 * params_.surface_circle.circularity_rmse_thresh);

    if (should_refine) {
      std::vector<Eigen::Vector3d> points3d;
      points3d.reserve(metric.vertex_ids.size());
      for (const uint32_t vid : metric.vertex_ids) {
        if (vid < vertices.size()) {
          points3d.push_back(vertices[vid]);
        }
      }
      const RimRefinementOutcome refined = refine_rim_center(
        points3d,
        base_center3d,
        z_dir,
        radius,
        request,
        params_);
      if (refined.used_refined_center) {
        refinement = refined;
      } else {
        refinement.low_confidence = refinement.low_confidence || refined.low_confidence;
        refinement.low_coverage = refinement.low_coverage || refined.low_coverage;
        refinement.large_shift = refined.large_shift;
        refinement.center_shift = refined.center_shift;
      }
    }

    const Eigen::Vector3d primary_seed = select_seed(params_.pose.x_seed, metric.axis_major, metric.axis_minor);
    const Eigen::Vector3d fallback_seed = select_seed(params_.pose.gram_schmidt_fallback, metric.axis_major, metric.axis_minor);
    const Frame frame = make_frame(z_dir, primary_seed, fallback_seed);

    msg::Hole hole;
    hole.kind = msg::Hole::SURFACE_CIRCLE;
    hole.diameter = static_cast<float>(2.0 * refinement.radius);
    hole.length = params_.surface_circle.has_thickness ?
      static_cast<float>(params_.surface_circle.thickness) : 0.0f;
    hole.pose.position.x = refinement.center.x();
    hole.pose.position.y = refinement.center.y();
    hole.pose.position.z = refinement.center.z();
    hole.pose.orientation = quaternion_from_frame(frame);
    hole.axis.x = frame.z.x();
    hole.axis.y = frame.z.y();
    hole.axis.z = frame.z.z();

    detections.push_back(std::move(hole));
    if (diagnostics) {
      ++diagnostics->detections_emitted;
      if (refinement.low_confidence) {
        ++diagnostics->low_confidence_count;
      }
      if (refinement.low_coverage) {
        ++diagnostics->low_coverage_count;
      }
      if (refinement.large_shift) {
        ++diagnostics->large_shift_count;
      }
    }
    if (qualities != nullptr) {
      qualities->push_back(to_hole_quality(refinement));
    }
  }

  return detections;
}


}  // namespace hole_toolpath_planner
