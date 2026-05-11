#include "hole_toolpath_planner/hole_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "hole_detection_common.hpp"

namespace hole_toolpath_planner
{
using detail::Frame;
using detail::fit_plane_pca;
using detail::make_frame;
using detail::normalize_or_default;
using detail::quaternion_from_frame;
using detail::select_seed;

std::vector<msg::Hole> HoleDetector::detect_solid_mode(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SolidDiagnostics * diagnostics,
  std::vector<HoleQuality> * qualities) const
{
  std::vector<msg::Hole> detections;

  pcl::PointCloud<pcl::PointXYZ> vertex_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_cloud);
  if (diagnostics) {
    diagnostics->attempted = true;
    diagnostics->vertex_count = vertex_cloud.size();
    diagnostics->polygon_count = mesh.polygons.size();
  }
  if (vertex_cloud.empty() || mesh.polygons.empty()) {
    if (diagnostics) {
      diagnostics->missing_vertices = vertex_cloud.empty();
      diagnostics->missing_polygons = mesh.polygons.empty();
    }
    RCLCPP_WARN_ONCE(logger_, "Solid-mode requested but mesh lacks vertices or polygons.");
    return detections;
  }

  struct TriangleSample
  {
    Eigen::Vector3d v0;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    double area;
  };

  std::vector<TriangleSample> triangles;
  triangles.reserve(mesh.polygons.size());

  const size_t vertex_count = vertex_cloud.size();
  bool warned_non_tri = false;
  for (const auto & poly : mesh.polygons) {
    if (poly.vertices.size() != 3) {
      if (!warned_non_tri) {
        RCLCPP_WARN(logger_, "Solid-mode currently supports triangle meshes only; ignoring non-triangles.");
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
      continue;
    }

    const Eigen::Vector3d v0{vertex_cloud[vids[0]].x, vertex_cloud[vids[0]].y, vertex_cloud[vids[0]].z};
    const Eigen::Vector3d v1{vertex_cloud[vids[1]].x, vertex_cloud[vids[1]].y, vertex_cloud[vids[1]].z};
    const Eigen::Vector3d v2{vertex_cloud[vids[2]].x, vertex_cloud[vids[2]].y, vertex_cloud[vids[2]].z};

    const Eigen::Vector3d e0 = v1 - v0;
    const Eigen::Vector3d e1 = v2 - v0;
    const double area2 = e0.cross(e1).norm();
    if (area2 < 1e-12) {
      continue;
    }

    TriangleSample tri{v0, v1, v2, 0.5 * area2};
    triangles.push_back(std::move(tri));
    if (diagnostics) {
      ++diagnostics->valid_triangles;
    }
  }

  if (triangles.empty()) {
    if (diagnostics) {
      diagnostics->no_valid_triangles = true;
    }
    RCLCPP_WARN(logger_, "Solid-mode sampling skipped: no valid triangles detected.");
    return detections;
  }

  const int sample_points = std::max(1, params_.sampling.points);
  if (diagnostics) {
    diagnostics->samples_requested = static_cast<size_t>(sample_points);
  }
  std::vector<double> weights;
  weights.reserve(triangles.size());
  double total_area = 0.0;
  for (const auto & tri : triangles) {
    weights.push_back(tri.area);
    total_area += tri.area;
  }

  if (total_area < 1e-9) {
    if (diagnostics) {
      diagnostics->area_too_small = true;
    }
    RCLCPP_WARN(logger_, "Solid-mode sampling failed: mesh surface area too small.");
    return detections;
  }

  std::mt19937 rng(static_cast<uint32_t>(params_.sampling.seed));
  std::discrete_distribution<size_t> tri_dist(weights.begin(), weights.end());
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr samples(new pcl::PointCloud<pcl::PointXYZ>());
  samples->reserve(static_cast<size_t>(sample_points));

  for (int i = 0; i < sample_points; ++i) {
    const TriangleSample & tri = triangles[tri_dist(rng)];
    const double r1 = unit_dist(rng);
    const double r2 = unit_dist(rng);
    const double sqrt_r1 = std::sqrt(r1);
    const double u = 1.0 - sqrt_r1;
    const double v = sqrt_r1 * (1.0 - r2);
    const double w = sqrt_r1 * r2;

    const Eigen::Vector3d point = u * tri.v0 + v * tri.v1 + w * tri.v2;
    samples->push_back(pcl::PointXYZ(static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())));
  }

  if (diagnostics) {
    diagnostics->samples_generated = samples->size();
  }

  const Eigen::Vector3d axis_hint = normalize_or_default(
    Eigen::Vector3d{
      params_.surface_circle.into_hint[0],
      params_.surface_circle.into_hint[1],
      params_.surface_circle.into_hint[2]},
    Eigen::Vector3d::UnitZ());
  const double axis_alignment_min = std::clamp(params_.cylinder_fit.axis_alignment_min, 0.0, 1.0);

  const size_t min_inliers_required = static_cast<size_t>(std::max(params_.cylinder_fit.min_inliers, 1));
  const double min_inlier_ratio = std::clamp(params_.cylinder_fit.min_inlier_ratio, 0.0, 1.0);
  if (diagnostics) {
    diagnostics->min_inliers_required = min_inliers_required;
    diagnostics->min_inlier_ratio = min_inlier_ratio;
  }

  if (samples->size() < min_inliers_required) {
    if (diagnostics) {
      diagnostics->insufficient_samples = true;
    }
    RCLCPP_WARN(logger_, "Solid-mode sampling produced insufficient points (%zu)", samples->size());
    return detections;
  }

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setInputCloud(samples);
  normal_est.setKSearch(std::max(3, params_.normals.k));
  normal_est.setViewPoint(
    static_cast<float>(params_.normals.viewpoint[0]),
    static_cast<float>(params_.normals.viewpoint[1]),
    static_cast<float>(params_.normals.viewpoint[2]));
  normal_est.compute(*normals);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*samples));
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>(*normals));

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

  const double radius_min = std::max(
    params_.cylinder_fit.radius_min,
    request_min_radius);
  double radius_max = params_.cylinder_fit.radius_max;
  if (request_max_radius > 0.0) {
    radius_max = std::min(radius_max, request_max_radius);
  }
  if (radius_min >= radius_max) {
    if (diagnostics) {
      diagnostics->invalid_radius_constraints = true;
    }
    RCLCPP_WARN(logger_, "Solid-mode radius constraints invalid (min %.4f >= max %.4f)", radius_min, radius_max);
    return detections;
  }

  const double min_length = std::max(0.0, static_cast<double>(request.min_length));

  const double r_scale = params_.pose.neighbor_radius_scale;
  const double neighbor_slab = params_.pose.neighbor_slab_thickness <= 0.0 ?
    params_.cylinder_fit.distance_threshold * 2.0 : params_.pose.neighbor_slab_thickness;

  while (cloud->size() >= min_inliers_required) {
    if (diagnostics) {
      ++diagnostics->segmentation_attempts;
    }
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(params_.cylinder_fit.normal_distance_weight);
    seg.setMaxIterations(params_.cylinder_fit.max_iterations);
    seg.setDistanceThreshold(params_.cylinder_fit.distance_threshold);
    seg.setRadiusLimits(radius_min, radius_max);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(inliers, coefficients);

    if (coefficients.values.size() < 7) {
      if (diagnostics) {
        diagnostics->last_inlier_count = inliers.indices.size();
        diagnostics->last_inlier_ratio = cloud->empty() ? 0.0 :
          static_cast<double>(inliers.indices.size()) / static_cast<double>(cloud->size());
      }
      break;
    }

    const size_t inlier_count = static_cast<size_t>(inliers.indices.size());
    const double inlier_ratio = cloud->empty() ? 0.0 :
      static_cast<double>(inlier_count) / static_cast<double>(cloud->size());
    if (diagnostics) {
      diagnostics->last_inlier_count = inlier_count;
      diagnostics->last_inlier_ratio = inlier_ratio;
    }

    const bool below_count_threshold = inlier_count < min_inliers_required;
    const bool below_ratio_threshold = inlier_ratio < min_inlier_ratio;
    if (below_count_threshold && below_ratio_threshold) {
      if (diagnostics) {
        ++diagnostics->cylinders_rejected_inliers;
      }
      RCLCPP_DEBUG(
        logger_,
        "Solid-mode cylinder candidate rejected: inliers=%zu (min=%zu), ratio=%.4f (min=%.4f)",
        inlier_count,
        min_inliers_required,
        inlier_ratio,
        min_inlier_ratio);
      break;
    }

    const Eigen::Vector3d axis_point{
      coefficients.values[0],
      coefficients.values[1],
      coefficients.values[2]};
    Eigen::Vector3d axis_dir{
      coefficients.values[3],
      coefficients.values[4],
      coefficients.values[5]};
    axis_dir = normalize_or_default(axis_dir, Eigen::Vector3d::UnitZ());
    const double radius = coefficients.values[6];

    std::vector<Eigen::Vector3d> inlier_points;
    inlier_points.reserve(inliers.indices.size());
    for (const int idx : inliers.indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= cloud->size()) {
        continue;
      }
      const auto & pt = cloud->points[static_cast<size_t>(idx)];
      inlier_points.emplace_back(pt.x, pt.y, pt.z);
    }

    if (inlier_points.size() < min_inliers_required) {
      break;
    }

    double t_min = std::numeric_limits<double>::max();
    double t_max = std::numeric_limits<double>::lowest();
    std::vector<double> t_values;
    t_values.reserve(inlier_points.size());
    for (const auto & p : inlier_points) {
      const double t = (p - axis_point).dot(axis_dir);
      t_values.push_back(t);
      t_min = std::min(t_min, t);
      t_max = std::max(t_max, t);
    }

    const Eigen::Vector3d c_min = axis_point + t_min * axis_dir;
    const Eigen::Vector3d c_max = axis_point + t_max * axis_dir;

    const double r_lower = 1.2 * radius;
    const double r_upper = 3.0 * radius;
    const double ring_radius = std::clamp(r_scale * radius, r_lower, r_upper);

    std::vector<Eigen::Vector3d> entry_ring;
    std::vector<Eigen::Vector3d> exit_ring;
    entry_ring.reserve(inlier_points.size());
    exit_ring.reserve(inlier_points.size());

    for (size_t idx = 0; idx < inlier_points.size(); ++idx) {
      const Eigen::Vector3d & p = inlier_points[idx];
      const double t = t_values[idx];
      const Eigen::Vector3d projection = axis_point + t * axis_dir;
      const Eigen::Vector3d radial = p - projection;
      const double radial_dist = radial.norm();
      if (radial_dist > ring_radius) {
        continue;
      }

      if (std::abs(t - t_min) <= neighbor_slab) {
        entry_ring.push_back(p);
      }
      if (std::abs(t - t_max) <= neighbor_slab) {
        exit_ring.push_back(p);
      }
    }

    Eigen::Vector3d entry_plane_center = c_min;
    Eigen::Vector3d entry_plane_normal = axis_dir;
    Eigen::Vector3d exit_plane_center = c_max;
    Eigen::Vector3d exit_plane_normal = -axis_dir;

    Eigen::Vector3d tmp_centroid;
    Eigen::Vector3d tmp_normal;
    if (fit_plane_pca(entry_ring, tmp_centroid, tmp_normal)) {
      entry_plane_center = tmp_centroid;
      entry_plane_normal = tmp_normal;
    }
    if (fit_plane_pca(exit_ring, tmp_centroid, tmp_normal)) {
      exit_plane_center = tmp_centroid;
      exit_plane_normal = tmp_normal;
    }

    double score_entry = axis_dir.dot(-entry_plane_normal);
    double score_exit = axis_dir.dot(-exit_plane_normal);
    int entry_index = score_entry >= score_exit ? 0 : 1;
    double best_score = std::max(score_entry, score_exit);
    Eigen::Vector3d z_dir = axis_dir;
    Eigen::Vector3d chosen_center = entry_index == 0 ? entry_plane_center : exit_plane_center;
    Eigen::Vector3d chosen_normal = entry_index == 0 ? entry_plane_normal : exit_plane_normal;

    if (best_score < 0.0) {
      z_dir = -axis_dir;
      score_entry = z_dir.dot(-entry_plane_normal);
      score_exit = z_dir.dot(-exit_plane_normal);
      entry_index = score_entry >= score_exit ? 0 : 1;
      best_score = std::max(score_entry, score_exit);
      chosen_center = entry_index == 0 ? entry_plane_center : exit_plane_center;
      chosen_normal = entry_index == 0 ? entry_plane_normal : exit_plane_normal;
    }

    const bool entry_plane_found = best_score >= 0.0;
    bool detection_valid = entry_plane_found;
    bool rejected_for_length = false;
    bool rejected_for_alignment = false;
    bool rejected_for_origin = false;
    double axis_alignment = 0.0;
    double length = 0.0;
    Eigen::Vector3d origin = chosen_center;

    if (detection_valid && chosen_normal.dot(z_dir) > 0.0) {
      chosen_normal = -chosen_normal;
    }

    if (detection_valid) {
      axis_alignment = std::abs(z_dir.dot(axis_hint));
      if (axis_alignment < axis_alignment_min) {
        detection_valid = false;
        rejected_for_alignment = true;
      }
    }

    if (detection_valid) {
      const double denom = z_dir.dot(chosen_normal);
      if (std::abs(denom) > 1e-9) {
        const double t_intersect = (chosen_center - axis_point).dot(chosen_normal) / denom;
        origin = axis_point + t_intersect * z_dir;
      }

      double new_t_min = std::numeric_limits<double>::max();
      double new_t_max = std::numeric_limits<double>::lowest();
      for (const auto & p : inlier_points) {
        const double t = (p - axis_point).dot(z_dir);
        new_t_min = std::min(new_t_min, t);
        new_t_max = std::max(new_t_max, t);
      }

      length = new_t_max - new_t_min;
      const double max_length = params_.cylinder_fit.length_max;
      if (length < min_length || !std::isfinite(length) ||
        (max_length > 0.0 && length > max_length))
      {
        detection_valid = false;
        rejected_for_length = true;
      }
    }

    if (detection_valid) {
      const double plane_offset = (origin - chosen_center).dot(chosen_normal);
      if (std::abs(plane_offset) > 2.0 * params_.cylinder_fit.distance_threshold) {
        origin -= plane_offset * chosen_normal;
      }
      if (
        !std::isfinite(origin.x()) ||
        !std::isfinite(origin.y()) ||
        !std::isfinite(origin.z()))
      {
        detection_valid = false;
        rejected_for_origin = true;
      }
    }

    if (detection_valid) {
      Eigen::Vector3d tangent_seed = Eigen::Vector3d::Zero();
      for (const auto & p : entry_ring) {
        const Eigen::Vector3d radial = p - origin - z_dir * ((p - origin).dot(z_dir));
        tangent_seed += radial;
      }
      if (tangent_seed.norm() < 1e-6) {
        tangent_seed = Eigen::Vector3d::UnitX();
      } else {
        tangent_seed.normalize();
      }
      Eigen::Vector3d minor_seed = z_dir.cross(tangent_seed);
      if (minor_seed.norm() < 1e-6) {
        minor_seed = z_dir.cross(Eigen::Vector3d::UnitY());
      }

      const Eigen::Vector3d primary_seed = select_seed(params_.pose.x_seed, tangent_seed, minor_seed);
      const Eigen::Vector3d fallback_seed = select_seed(params_.pose.gram_schmidt_fallback, tangent_seed, minor_seed);
      const Frame frame = make_frame(z_dir, primary_seed, fallback_seed);

      msg::Hole hole;
      hole.kind = msg::Hole::CYLINDER;
      const double diameter = 2.0 * radius;
      hole.diameter = static_cast<float>(diameter);
      hole.length = static_cast<float>(length);
      hole.pose.position.x = origin.x();
      hole.pose.position.y = origin.y();
      hole.pose.position.z = origin.z();
      hole.pose.orientation = quaternion_from_frame(frame);
      hole.axis.x = frame.z.x();
      hole.axis.y = frame.z.y();
      hole.axis.z = frame.z.z();

      detections.push_back(std::move(hole));
      if (qualities != nullptr) {
        qualities->push_back(HoleQuality{});
      }
      if (diagnostics) {
        ++diagnostics->cylinders_emitted;
      }
      RCLCPP_INFO(
        logger_,
        "Solid-mode cylinder accepted: diameter=%.4f m, length=%.4f m, center=(%.4f, %.4f, %.4f), "
        "inliers=%zu (ratio=%.4f), alignment=%.3f",
        diameter,
        length,
        origin.x(),
        origin.y(),
        origin.z(),
        inlier_count,
        inlier_ratio,
        axis_alignment);
    }

    if (!detection_valid) {
      RCLCPP_DEBUG(
        logger_,
        "Solid-mode cylinder rejected after validation: entry_plane_found=%d, rejected_for_length=%d, "
        "rejected_for_alignment=%d, rejected_for_origin=%d, alignment=%.3f, inliers=%zu (ratio=%.4f)",
        entry_plane_found ? 1 : 0,
        rejected_for_length ? 1 : 0,
        rejected_for_alignment ? 1 : 0,
        rejected_for_origin ? 1 : 0,
        axis_alignment,
        inlier_count,
        inlier_ratio);
    }

    if (!detection_valid && diagnostics) {
      if (!entry_plane_found) {
        ++diagnostics->cylinders_no_entry_plane;
      } else if (rejected_for_alignment) {
        ++diagnostics->cylinders_rejected_alignment;
      } else if (rejected_for_length) {
        ++diagnostics->cylinders_rejected_length;
      } else if (rejected_for_origin) {
        ++diagnostics->cylinders_invalid_origin;
      }
    }

    if (!params_.cylinder_fit.extract_iteratively) {
      break;
    }

    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
    pcl::ExtractIndices<pcl::PointXYZ> extract_points;
    extract_points.setInputCloud(cloud);
    extract_points.setIndices(inliers_ptr);
    extract_points.setNegative(true);
    extract_points.filter(*cloud);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_ptr);
    extract_normals.setNegative(true);
    extract_normals.filter(*cloud_normals);
  }

  return detections;
}


}  // namespace hole_toolpath_planner
