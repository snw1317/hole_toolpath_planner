#include "hole_toolpath_planner/hole_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "hole_toolpath_planner/toolpath_generator.hpp"

namespace hole_toolpath_planner
{
namespace
{
rclcpp::Logger::Level level_from_string(const std::string & value)
{
  if (value == "debug") {
    return rclcpp::Logger::Level::Debug;
  }
  if (value == "warn") {
    return rclcpp::Logger::Level::Warn;
  }
  if (value == "error") {
    return rclcpp::Logger::Level::Error;
  }
  if (value == "fatal") {
    return rclcpp::Logger::Level::Fatal;
  }
  return rclcpp::Logger::Level::Info;
}

double to_radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double axis_angle(const geometry_msgs::msg::Vector3 & a, const geometry_msgs::msg::Vector3 & b)
{
  const Eigen::Vector3d va{a.x, a.y, a.z};
  const Eigen::Vector3d vb{b.x, b.y, b.z};
  const double norm_product = va.norm() * vb.norm();
  if (norm_product < 1e-9) {
    return 0.0;
  }
  const double cos_angle = std::clamp(va.dot(vb) / norm_product, -1.0, 1.0);
  return std::acos(cos_angle);
}

double distance_between(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const Eigen::Vector3d va{a.x, a.y, a.z};
  const Eigen::Vector3d vb{b.x, b.y, b.z};
  return (va - vb).norm();
}

Eigen::Vector3d normalize_or_default(const Eigen::Vector3d & v, const Eigen::Vector3d & fallback)
{
  const double norm = v.norm();
  if (norm > 1e-9) {
    return v / norm;
  }
  return fallback;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[mid];
  }
  if (mid == 0) {
    return values[0];
  }
  return 0.5 * (values[mid - 1] + values[mid]);
}

struct CylinderFitSimple
{
  bool valid{false};
  double radius{0.0};
  double length{0.0};
  Eigen::Vector3d top{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
};

CylinderFitSimple fit_cylinder_from_points(const std::vector<Eigen::Vector3d> & points)
{
  CylinderFitSimple result;
  if (points.size() < 6) {
    return result;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());

  Eigen::MatrixXd centered(points.size(), 3);
  for (size_t i = 0; i < points.size(); ++i) {
    centered.row(i) = (points[i] - mean).transpose();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  if (svd.matrixV().cols() < 3) {
    return result;
  }

  const Eigen::Matrix3d V = svd.matrixV();
  int best_index = 0;
  double best_abs_z = -1.0;
  for (int i = 0; i < 3; ++i) {
    const double abs_z = std::abs(V(2, i));
    if (abs_z > best_abs_z) {
      best_abs_z = abs_z;
      best_index = i;
    }
  }

  Eigen::Vector3d axis_dir = normalize_or_default(V.col(best_index), Eigen::Vector3d::UnitZ());

  std::vector<double> radial_distances;
  radial_distances.reserve(points.size());
  double t_min = std::numeric_limits<double>::infinity();
  double t_max = -std::numeric_limits<double>::infinity();

  for (const auto & p : points) {
    const Eigen::Vector3d diff = p - mean;
    const double t = diff.dot(axis_dir);
    t_min = std::min(t_min, t);
    t_max = std::max(t_max, t);
    const Eigen::Vector3d axis_point = mean + t * axis_dir;
    const double radial = (p - axis_point).norm();
    if (std::isfinite(radial)) {
      radial_distances.push_back(radial);
    }
  }

  if (!std::isfinite(t_min) || !std::isfinite(t_max) || radial_distances.size() < 3) {
    return result;
  }

  const double radius = median(radial_distances);
  if (!std::isfinite(radius) || radius <= 0.0) {
    return result;
  }

  Eigen::Vector3d min_point = mean + t_min * axis_dir;
  Eigen::Vector3d max_point = mean + t_max * axis_dir;

  Eigen::Vector3d top = max_point;
  Eigen::Vector3d bottom = min_point;
  if (top.z() < bottom.z()) {
    std::swap(top, bottom);
  }

  Eigen::Vector3d axis_vec = bottom - top;
  const double axis_norm = axis_vec.norm();
  if (axis_norm < 1e-9) {
    return result;
  }
  axis_vec /= axis_norm;

  if (axis_vec.z() > 0.0) {
    axis_vec = -axis_vec;
    std::swap(top, bottom);
  }

  const double length = (bottom - top).norm();
  if (!std::isfinite(length) || length < 1e-6) {
    return result;
  }

  result.valid = true;
  result.radius = radius;
  result.length = length;
  result.top = top;
  result.bottom = bottom;
  result.axis = axis_vec;
  return result;
}

struct Frame
{
  Eigen::Vector3d x;
  Eigen::Vector3d y;
  Eigen::Vector3d z;
};

Frame make_frame(
  const Eigen::Vector3d & z_dir,
  const Eigen::Vector3d & primary_seed,
  const Eigen::Vector3d & fallback_seed)
{
  Eigen::Vector3d z = normalize_or_default(z_dir, Eigen::Vector3d::UnitZ());

  auto orthogonalize = [&z](const Eigen::Vector3d & seed) -> Eigen::Vector3d {
    Eigen::Vector3d projected = seed - seed.dot(z) * z;
    const double norm = projected.norm();
    if (norm < 1e-9) {
      return Eigen::Vector3d::Zero();
    }
    return projected / norm;
  };

  Eigen::Vector3d x = orthogonalize(primary_seed);
  if (x.isZero(1e-9)) {
    x = orthogonalize(fallback_seed);
  }
  if (x.isZero(1e-9)) {
    // Last resort: pick an arbitrary axis orthogonal to z.
    if (std::abs(z.dot(Eigen::Vector3d::UnitX())) < 0.9) {
      x = orthogonalize(Eigen::Vector3d::UnitX());
    } else {
      x = orthogonalize(Eigen::Vector3d::UnitY());
    }
  }

  Eigen::Vector3d y = z.cross(x);
  const double y_norm = y.norm();
  if (y_norm < 1e-9) {
    y = z.cross(Eigen::Vector3d::UnitZ());
  } else {
    y /= y_norm;
  }
  x = y.cross(z);
  x.normalize();

  return Frame{x, y, z};
}

geometry_msgs::msg::Quaternion quaternion_from_frame(const Frame & frame)
{
  Eigen::Matrix3d rotation;
  rotation.col(0) = frame.x;
  rotation.col(1) = frame.y;
  rotation.col(2) = frame.z;
  Eigen::Quaterniond q(rotation);
  q.normalize();
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

struct CircleFitResult
{
  bool success{false};
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double rmse{std::numeric_limits<double>::infinity()};
};

CircleFitResult fit_circle_pratt(const std::vector<Eigen::Vector2d> & points)
{
  CircleFitResult result;
  if (points.size() < 3) {
    return result;
  }

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());

  double Suu = 0.0;
  double Suv = 0.0;
  double Svv = 0.0;
  double Suuu = 0.0;
  double Suvv = 0.0;
  double Svvv = 0.0;
  double Svuu = 0.0;

  for (const auto & p : points) {
    const double u = p.x() - mean.x();
    const double v = p.y() - mean.y();
    const double uu = u * u;
    const double vv = v * v;

    Suu += uu;
    Svv += vv;
    Suv += u * v;
    Suuu += uu * u;
    Svvv += vv * v;
    Suvv += u * vv;
    Svuu += v * uu;
  }

  const double det = 2.0 * (Suu * Svv - Suv * Suv);
  if (std::abs(det) < 1e-12) {
    return result;
  }

  const double uc = (Svv * (Suuu + Suvv) - Suv * (Svvv + Svuu)) / det;
  const double vc = (Suu * (Svvv + Svuu) - Suv * (Suuu + Suvv)) / det;

  const Eigen::Vector2d center = mean + Eigen::Vector2d(uc, vc);

  double radius_acc = 0.0;
  double rmse_acc = 0.0;
  for (const auto & p : points) {
    const double dist = (p - center).norm();
    radius_acc += dist;
  }
  const double radius = radius_acc / static_cast<double>(points.size());

  for (const auto & p : points) {
    const double dist = (p - center).norm();
    const double residual = dist - radius;
    rmse_acc += residual * residual;
  }

  result.success = true;
  result.center = center;
  result.radius = radius;
  result.rmse = std::sqrt(rmse_acc / static_cast<double>(points.size()));
  return result;
}

double polygon_area_2d(const std::vector<Eigen::Vector2d> & points)
{
  if (points.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % points.size()];
    area += a.x() * b.y() - b.x() * a.y();
  }
  return 0.5 * area;
}

uint64_t edge_key(uint32_t a, uint32_t b)
{
  const uint32_t lo = std::min(a, b);
  const uint32_t hi = std::max(a, b);
  return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

struct EdgeInfo
{
  uint32_t first_face{std::numeric_limits<uint32_t>::max()};
  uint32_t second_face{std::numeric_limits<uint32_t>::max()};
  std::pair<uint32_t, uint32_t> first_oriented{0U, 0U};
  std::pair<uint32_t, uint32_t> second_oriented{0U, 0U};
};

struct BoundaryEdge
{
  uint32_t start;
  uint32_t end;
  uint32_t face;
  int component;
  bool visited{false};
};

Eigen::Vector3d select_seed(
  const std::string & key,
  const Eigen::Vector3d & major_axis,
  const Eigen::Vector3d & minor_axis)
{
  if (key == "world_x") {
    return Eigen::Vector3d::UnitX();
  }
  if (key == "world_y") {
    return Eigen::Vector3d::UnitY();
  }
  if (key == "world_z") {
    return Eigen::Vector3d::UnitZ();
  }
  if (key == "plane_minor") {
    return minor_axis;
  }
  if (key == "mesh_x" || key == "plane_major") {
    return major_axis;
  }
  return Eigen::Vector3d::UnitX();
}

bool fit_plane_pca(
  const std::vector<Eigen::Vector3d> & points,
  Eigen::Vector3d & centroid,
  Eigen::Vector3d & normal)
{
  if (points.size() < 3) {
    return false;
  }

  centroid = Eigen::Vector3d::Zero();
  for (const auto & p : points) {
    centroid += p;
  }
  centroid /= static_cast<double>(points.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & p : points) {
    const Eigen::Vector3d diff = p - centroid;
    covariance += diff * diff.transpose();
  }
  covariance /= static_cast<double>(points.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  normal = normalize_or_default(solver.eigenvectors().col(0), Eigen::Vector3d::UnitZ());
  return true;
}
}  // namespace

HoleDetector::HoleDetector(rclcpp::Node & node, PlannerParameters params)
: node_(node), params_(std::move(params)), logger_(node.get_logger())
{
  logger_.set_level(level_from_string(params_.logging.min_severity));
}

msg::HoleArray HoleDetector::detect(const srv::DetectHoles::Request & request)
{
  const rclcpp::Time stamp = node_.get_clock()->now();
  pcl::PolygonMesh mesh;

  if (!load_mesh(request.mesh_path, mesh)) {
    RCLCPP_WARN(logger_, "Failed to load mesh '%s'", request.mesh_path.c_str());
    return assemble_response({}, stamp);
  }

  std::vector<msg::Hole> holes;
  holes.reserve(32);

  const bool do_surface = params_.detection.mode == "surface" || params_.detection.mode == "auto";
  const bool do_solid = params_.detection.mode == "solid" || params_.detection.mode == "auto";
  const bool run_solid = do_solid || request.watertight_hint;

  SurfaceDiagnostics surface_diag;
  SolidDiagnostics solid_diag;

  if (do_surface) {
    auto surface_holes = detect_surface_clusters(mesh, request, &surface_diag);
    holes.insert(holes.end(), surface_holes.begin(), surface_holes.end());
    if (surface_holes.empty()) {
      auto legacy_surface = detect_surface_mode_legacy(mesh, request, &surface_diag);
      holes.insert(holes.end(), legacy_surface.begin(), legacy_surface.end());
    }
  }

  if (run_solid) {
    auto solid_holes = detect_solid_mode(mesh, request, &solid_diag);
    holes.insert(holes.end(), solid_holes.begin(), solid_holes.end());
  }

  const size_t before_dedup = holes.size();
  auto deduped = deduplicate(std::move(holes));

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
         << ", outer_skipped=" << surface_diag.loops_outer_skipped << ")";
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

std::vector<msg::Hole> HoleDetector::detect_surface_clusters(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SurfaceDiagnostics * diagnostics) const
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
  constexpr double cluster_tolerance = 0.01;
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

    const Frame frame = make_frame(fit.axis, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());

    msg::Hole hole;
    hole.kind = msg::Hole::SURFACE_CIRCLE;
    hole.diameter = static_cast<float>(diameter);
    hole.length = static_cast<float>(fit.length);
    hole.pose.position.x = fit.top.x();
    hole.pose.position.y = fit.top.y();
    hole.pose.position.z = fit.top.z();
    hole.pose.orientation = quaternion_from_frame(frame);
    hole.axis.x = frame.z.x();
    hole.axis.y = frame.z.y();
    hole.axis.z = frame.z.z();

    detections.push_back(std::move(hole));

    if (diagnostics) {
      ++diagnostics->detections_emitted;
      ++diagnostics->circle_fit_success;
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

std::vector<msg::Hole> HoleDetector::detect_surface_mode_legacy(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SurfaceDiagnostics * diagnostics) const
{
  if (diagnostics) {
    diagnostics->attempted = true;
    diagnostics->face_count = mesh.polygons.size();
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

    const Eigen::Vector3d center3d = metric.centroid +
      metric.axis_major * metric.circle.center.x() +
      metric.axis_minor * metric.circle.center.y();

    const Eigen::Vector3d primary_seed = select_seed(params_.pose.x_seed, metric.axis_major, metric.axis_minor);
    const Eigen::Vector3d fallback_seed = select_seed(params_.pose.gram_schmidt_fallback, metric.axis_major, metric.axis_minor);
    const Frame frame = make_frame(z_dir, primary_seed, fallback_seed);

    msg::Hole hole;
    hole.kind = msg::Hole::SURFACE_CIRCLE;
    hole.diameter = static_cast<float>(diameter);
    hole.length = params_.surface_circle.has_thickness ?
      static_cast<float>(params_.surface_circle.thickness) : 0.0f;
    hole.pose.position.x = center3d.x();
    hole.pose.position.y = center3d.y();
    hole.pose.position.z = center3d.z();
    hole.pose.orientation = quaternion_from_frame(frame);
    hole.axis.x = frame.z.x();
    hole.axis.y = frame.z.y();
    hole.axis.z = frame.z.z();

    detections.push_back(std::move(hole));
    if (diagnostics) {
      ++diagnostics->detections_emitted;
    }
  }

  return detections;
}

std::vector<msg::Hole> HoleDetector::detect_solid_mode(
  const pcl::PolygonMesh & mesh,
  const srv::DetectHoles::Request & request,
  SolidDiagnostics * diagnostics) const
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

std::vector<msg::Hole> HoleDetector::deduplicate(std::vector<msg::Hole> && holes) const
{
  if (holes.size() <= 1) {
    return holes;
  }

  const double angle_thresh = to_radians(params_.detection.dedupe_angle_deg);
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

      const double angle = axis_angle(holes[i].axis, holes[j].axis);
      if (angle > angle_thresh) {
        continue;
      }

      if (distance_between(holes[i].pose.position, holes[j].pose.position) > center_tol) {
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
      hole.pose.orientation.w * hole.pose.orientation.w
    );
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
