#include "hole_toolpath_planner/hole_detector.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <Eigen/Core>
#include <pcl/io/vtk_lib_io.h>

#include "hole_detection_common.hpp"
#include "hole_toolpath_planner/toolpath_generator.hpp"

namespace hole_toolpath_planner
{
HoleDetector::HoleDetector(rclcpp::Node & node, PlannerParameters params)
: node_(node), params_(std::move(params)), logger_(node.get_logger())
{
  logger_.set_level(detail::level_from_string(params_.logging.min_severity));
}

msg::HoleArray HoleDetector::detect(const srv::DetectHoles::Request & request)
{
  last_detection_quality_.clear();
  const rclcpp::Time stamp = node_.get_clock()->now();
  pcl::PolygonMesh mesh;

  if (!load_mesh(request.mesh_path, mesh)) {
    RCLCPP_WARN(logger_, "Failed to load mesh '%s'", request.mesh_path.c_str());
    return assemble_response({}, stamp);
  }

  std::vector<msg::Hole> holes;
  holes.reserve(32);
  std::vector<HoleQuality> hole_quality;
  hole_quality.reserve(32);

  const bool do_surface = params_.detection.mode == "surface" || params_.detection.mode == "auto";
  const bool do_solid = params_.detection.mode == "solid" || params_.detection.mode == "auto";
  const bool run_solid = do_solid || request.watertight_hint;

  SurfaceDiagnostics surface_diag;
  SolidDiagnostics solid_diag;

  if (do_surface) {
    const std::string strategy = params_.detection.surface_strategy;
    if (strategy == "legacy") {
      auto legacy_surface = detect_surface_mode_legacy(mesh, request, &surface_diag, &hole_quality);
      holes.insert(holes.end(), legacy_surface.begin(), legacy_surface.end());
    } else if (strategy == "radial") {
      auto radial_surface = detect_surface_clusters_radial(mesh, request, &surface_diag, &hole_quality);
      holes.insert(holes.end(), radial_surface.begin(), radial_surface.end());
    } else if (strategy == "clusters") {
      auto surface_holes = detect_surface_clusters(mesh, request, &surface_diag, &hole_quality);
      holes.insert(holes.end(), surface_holes.begin(), surface_holes.end());
    } else {
      auto surface_holes = detect_surface_clusters(mesh, request, &surface_diag, &hole_quality);
      holes.insert(holes.end(), surface_holes.begin(), surface_holes.end());
      if (surface_holes.empty()) {
        auto legacy_surface = detect_surface_mode_legacy(mesh, request, &surface_diag, &hole_quality);
        holes.insert(holes.end(), legacy_surface.begin(), legacy_surface.end());
      }
    }
  }

  if (run_solid) {
    auto solid_holes = detect_solid_mode(mesh, request, &solid_diag, &hole_quality);
    holes.insert(holes.end(), solid_holes.begin(), solid_holes.end());
  }

  const size_t before_dedup = holes.size();
  std::vector<size_t> kept_indices;
  kept_indices.reserve(before_dedup);
  auto deduped = deduplicate(std::move(holes), &kept_indices);
  last_detection_quality_.reserve(deduped.size());
  for (const size_t idx : kept_indices) {
    if (idx < hole_quality.size()) {
      last_detection_quality_.push_back(hole_quality[idx]);
    } else {
      last_detection_quality_.push_back(HoleQuality{});
    }
  }
  if (last_detection_quality_.size() != deduped.size()) {
    last_detection_quality_.assign(deduped.size(), HoleQuality{});
  }

  if (deduped.empty()) {
    RCLCPP_WARN(logger_, "No holes detected in mesh '%s'", request.mesh_path.c_str());
    if (do_surface && surface_diag.attempted) {
      std::ostringstream ss;
      ss << "Surface-mode diagnostics: faces=" << surface_diag.face_count
         << ", boundary_edges=" << surface_diag.boundary_edge_count
         << ", loops_valid=" << surface_diag.loops_total
         << ", considered=" << surface_diag.loops_considered
         << ", circle_fit_success=" << surface_diag.circle_fit_success
         << ", emitted=" << surface_diag.detections_emitted
         << "; rejects(circle_fit=" << surface_diag.circle_fit_failures
         << ", rmse=" << surface_diag.rmse_rejections
         << ", radius=" << surface_diag.radius_rejections
         << ", topology=" << surface_diag.loops_invalid_topology
         << ", too_small=" << surface_diag.loops_too_small
         << ", invalid_vertices=" << surface_diag.loops_invalid_vertices
         << ", outer_skipped=" << surface_diag.loops_outer_skipped
         << ", low_confidence=" << surface_diag.low_confidence_count
         << ", low_coverage=" << surface_diag.low_coverage_count
         << ", large_shift=" << surface_diag.large_shift_count << ")";
      if (surface_diag.mesh_empty) {
        ss << "; mesh contained no polygons";
      }
      if (surface_diag.vertices_empty) {
        ss << "; mesh contained no vertices";
      }
      if (surface_diag.boundary_edges_empty) {
        ss << "; no boundary edges were detected";
      }
      RCLCPP_WARN(logger_, "%s", ss.str().c_str());
    }
    if (run_solid && solid_diag.attempted) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(4);
      ss << "Solid-mode diagnostics: vertices=" << solid_diag.vertex_count
         << ", polygons=" << solid_diag.polygon_count
         << ", valid_triangles=" << solid_diag.valid_triangles
         << ", samples(requested=" << solid_diag.samples_requested
         << ", generated=" << solid_diag.samples_generated << ")"
         << ", segmentations=" << solid_diag.segmentation_attempts
         << ", emitted=" << solid_diag.cylinders_emitted
         << "; rejects(no_entry_plane=" << solid_diag.cylinders_no_entry_plane
         << ", alignment=" << solid_diag.cylinders_rejected_alignment
         << ", length=" << solid_diag.cylinders_rejected_length
         << ", origin=" << solid_diag.cylinders_invalid_origin
         << ", inliers=" << solid_diag.cylinders_rejected_inliers << ")"
         << "; thresholds(min_inliers=" << solid_diag.min_inliers_required
         << ", min_ratio=" << solid_diag.min_inlier_ratio
         << ", axis_align_min=" << params_.cylinder_fit.axis_alignment_min
         << ", length_max=" << params_.cylinder_fit.length_max << ")";
      if (solid_diag.last_inlier_count > 0 || solid_diag.last_inlier_ratio > 0.0) {
        ss << "; last_candidate(inliers=" << solid_diag.last_inlier_count
           << ", ratio=" << solid_diag.last_inlier_ratio << ")";
      }
      if (solid_diag.missing_vertices) {
        ss << "; mesh provided no vertices";
      }
      if (solid_diag.missing_polygons) {
        ss << "; mesh provided no polygons";
      }
      if (solid_diag.no_valid_triangles) {
        ss << "; no valid triangles for sampling";
      }
      if (solid_diag.area_too_small) {
        ss << "; total triangle area below threshold";
      }
      if (solid_diag.insufficient_samples) {
        ss << "; insufficient sample points for RANSAC";
      }
      if (solid_diag.invalid_radius_constraints) {
        ss << "; radius constraints invalid";
      }
      RCLCPP_WARN(logger_, "%s", ss.str().c_str());
    }
  } else {
    std::ostringstream header;
    header << "Detected " << deduped.size() << " hole(s)";
    if (before_dedup != deduped.size()) {
      header << " (" << before_dedup << " before deduplication)";
    }
    header << " in mesh '" << request.mesh_path << "'";
    RCLCPP_INFO(logger_, "%s", header.str().c_str());

    for (size_t idx = 0; idx < deduped.size(); ++idx) {
      const auto & hole = deduped[idx];
      RCLCPP_INFO(
        logger_,
        "  [%02zu] center=(%.4f, %.4f, %.4f) m, diameter=%.3f mm, length=%.3f mm",
        idx,
        hole.pose.position.x,
        hole.pose.position.y,
        hole.pose.position.z,
        hole.diameter * 1000.0,
        hole.length * 1000.0);
    }
  }

  if (do_surface && surface_diag.attempted && surface_diag.low_confidence_count > 0) {
    RCLCPP_WARN(
      logger_,
      "Surface-mode quality warnings: low_confidence=%zu, low_coverage=%zu, large_shift=%zu",
      surface_diag.low_confidence_count,
      surface_diag.low_coverage_count,
      surface_diag.large_shift_count);
  }

  for (size_t idx = 0; idx < deduped.size() && idx < last_detection_quality_.size(); ++idx) {
    const auto & quality = last_detection_quality_[idx];
    if (!quality.low_confidence) {
      continue;
    }
    const auto & hole = deduped[idx];
    RCLCPP_WARN(
      logger_,
      "Low-confidence hole[%zu]: center=(%.4f, %.4f, %.4f) m, diameter=%.3f mm, coverage=%.2f, rmse=%.5f, shift=%.5f",
      idx,
      hole.pose.position.x,
      hole.pose.position.y,
      hole.pose.position.z,
      hole.diameter * 1000.0,
      static_cast<double>(quality.coverage),
      static_cast<double>(quality.rmse),
      static_cast<double>(quality.center_shift));
  }

  return assemble_response(std::move(deduped), stamp);
}

std::vector<msg::Toolpath> HoleDetector::make_toolpaths(
  const msg::HoleArray & holes,
  const rclcpp::Time & stamp)
{
  return build_toolpaths(holes, params_.toolpath, stamp, params_.logging.frame_id);
}

bool HoleDetector::load_mesh(const std::string & mesh_path, pcl::PolygonMesh & mesh) const
{
  std::error_code ec;
  if (!std::filesystem::exists(mesh_path, ec)) {
    RCLCPP_WARN(logger_, "Mesh path '%s' does not exist", mesh_path.c_str());
    return false;
  }

  const int polygons = pcl::io::loadPolygonFile(mesh_path, mesh);
  if (polygons <= 0) {
    RCLCPP_WARN(logger_, "Unable to parse mesh '%s'", mesh_path.c_str());
    return false;
  }

  if (mesh.polygons.empty()) {
    RCLCPP_WARN(logger_, "Mesh '%s' contains no polygons", mesh_path.c_str());
    return false;
  }

  return true;
}

std::vector<msg::Hole> HoleDetector::deduplicate(
  std::vector<msg::Hole> && holes,
  std::vector<size_t> * kept_source_indices) const
{
  if (holes.size() <= 1) {
    if (kept_source_indices != nullptr && !holes.empty()) {
      kept_source_indices->push_back(0);
    }
    return holes;
  }

  Eigen::Vector3d mean_center = Eigen::Vector3d::Zero();
  for (const auto & hole : holes) {
    mean_center += Eigen::Vector3d{hole.pose.position.x, hole.pose.position.y, hole.pose.position.z};
  }
  mean_center /= static_cast<double>(holes.size());

  const double angle_thresh = detail::to_radians(params_.detection.dedupe_angle_deg);
  const double center_tol = params_.detection.dedupe_center_tol;
  const double min_radius_tol = params_.detection.dedupe_radius_tol;

  std::vector<bool> keep(holes.size(), true);

  for (size_t i = 0; i < holes.size(); ++i) {
    if (!keep[i]) {
      continue;
    }

    for (size_t j = i + 1; j < holes.size(); ++j) {
      if (!keep[j]) {
        continue;
      }

      const double angle = detail::axis_angle(holes[i].axis, holes[j].axis);
      if (angle > angle_thresh) {
        continue;
      }

      if (detail::distance_between(holes[i].pose.position, holes[j].pose.position) > center_tol) {
        continue;
      }

      const double diameter_a = holes[i].diameter;
      const double diameter_b = holes[j].diameter;
      const double diameter_diff = std::fabs(diameter_a - diameter_b);
      const double min_diameter_tol = 2.0 * min_radius_tol;
      const double diameter_tol = std::max(min_diameter_tol, 0.02 * std::min(diameter_a, diameter_b));
      if (diameter_diff > diameter_tol) {
        continue;
      }

      const Eigen::Vector3d pi{holes[i].pose.position.x, holes[i].pose.position.y, holes[i].pose.position.z};
      const Eigen::Vector3d pj{holes[j].pose.position.x, holes[j].pose.position.y, holes[j].pose.position.z};
      Eigen::Vector3d axis{
        holes[i].axis.x,
        holes[i].axis.y,
        holes[i].axis.z};
      axis = detail::normalize_or_default(axis, Eigen::Vector3d::UnitZ());

      const Eigen::Vector3d d = pj - pi;
      const double axial_dist = std::abs(d.dot(axis));
      const double perp_dist = (d - d.dot(axis) * axis).norm();
      const double perp_tol = std::max(center_tol, 0.1 * std::min(diameter_a, diameter_b));
      const double length_i = static_cast<double>(holes[i].length);
      const double length_j = static_cast<double>(holes[j].length);
      const double length_tol = std::max(length_i, length_j) + 0.5 * std::max(diameter_a, diameter_b);

      if (perp_dist <= perp_tol && length_tol > 0.0 && axial_dist <= length_tol) {
        const double di = (pi - mean_center).norm();
        const double dj = (pj - mean_center).norm();
        const bool prefer_j = dj > di;
        if (prefer_j) {
          keep[i] = false;
          break;
        }
        keep[j] = false;
        continue;
      }

      const bool prefer_b = holes[j].kind == msg::Hole::CYLINDER && holes[i].kind != msg::Hole::CYLINDER;
      if (prefer_b) {
        keep[i] = false;
        break;
      }
      keep[j] = false;
    }
  }

  std::vector<msg::Hole> result;
  result.reserve(holes.size());
  for (size_t i = 0; i < holes.size(); ++i) {
    if (keep[i]) {
      if (kept_source_indices != nullptr) {
        kept_source_indices->push_back(i);
      }
      result.push_back(std::move(holes[i]));
    }
  }
  return result;
}

msg::HoleArray HoleDetector::assemble_response(
  std::vector<msg::Hole> && holes,
  const rclcpp::Time & stamp) const
{
  msg::HoleArray array;
  array.header.stamp = stamp;
  array.header.frame_id = params_.logging.frame_id;

  int32_t next_id = 0;
  for (auto & hole : holes) {
    hole.header.stamp = stamp;
    hole.header.frame_id = params_.logging.frame_id;
    hole.id = next_id++;

    Eigen::Vector3d axis{hole.axis.x, hole.axis.y, hole.axis.z};
    if (axis.norm() > 1e-9) {
      axis.normalize();
    } else {
      axis = Eigen::Vector3d::UnitZ();
    }
    hole.axis.x = axis.x();
    hole.axis.y = axis.y();
    hole.axis.z = axis.z();

    const double q_norm = std::sqrt(
      hole.pose.orientation.x * hole.pose.orientation.x +
      hole.pose.orientation.y * hole.pose.orientation.y +
      hole.pose.orientation.z * hole.pose.orientation.z +
      hole.pose.orientation.w * hole.pose.orientation.w);
    if (q_norm > 1e-9) {
      hole.pose.orientation.x /= q_norm;
      hole.pose.orientation.y /= q_norm;
      hole.pose.orientation.z /= q_norm;
      hole.pose.orientation.w /= q_norm;
    } else {
      hole.pose.orientation.x = 0.0;
      hole.pose.orientation.y = 0.0;
      hole.pose.orientation.z = 0.0;
      hole.pose.orientation.w = 1.0;
    }

    array.holes.push_back(std::move(hole));
  }

  return array;
}

}  // namespace hole_toolpath_planner
