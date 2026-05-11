#include "hole_detection_common.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace hole_toolpath_planner::detail
{
namespace
{
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

struct CircleModel2d
{
  bool valid{false};
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
};

struct CylinderCandidate
{
  bool valid{false};
  Eigen::Vector3d axis_dir{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d axis_point{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double length{0.0};
  double t_min{0.0};
  double t_max{0.0};
  double radial_rmse{std::numeric_limits<double>::infinity()};
};

CircleModel2d circle_from_3_points(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c)
{
  CircleModel2d model;
  const double ax = a.x();
  const double ay = a.y();
  const double bx = b.x();
  const double by = b.y();
  const double cx = c.x();
  const double cy = c.y();

  const double a1 = bx - ax;
  const double b1 = by - ay;
  const double c1 = cx - ax;
  const double d1 = cy - ay;

  const double e = a1 * (ax + bx) + b1 * (ay + by);
  const double f = c1 * (ax + cx) + d1 * (ay + cy);
  const double g = 2.0 * (a1 * (cy - by) - b1 * (cx - bx));

  if (std::abs(g) < 1e-12) {
    return model;
  }

  const double cx_center = (d1 * e - b1 * f) / g;
  const double cy_center = (a1 * f - c1 * e) / g;
  const Eigen::Vector2d center(cx_center, cy_center);
  const double radius = (center - a).norm();

  if (!std::isfinite(radius) || radius <= 0.0) {
    return model;
  }

  model.valid = true;
  model.center = center;
  model.radius = radius;
  return model;
}

double circle_rmse(
  const std::vector<Eigen::Vector2d> & points,
  const Eigen::Vector2d & center,
  const double radius)
{
  if (points.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double rmse_acc = 0.0;
  for (const auto & p : points) {
    const double dist = (p - center).norm();
    const double residual = dist - radius;
    rmse_acc += residual * residual;
  }
  return std::sqrt(rmse_acc / static_cast<double>(points.size()));
}

CylinderCandidate fit_cylinder_axis_candidate(
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Vector3d & mean,
  const Eigen::Vector3d & axis_seed)
{
  CylinderCandidate candidate;
  const Eigen::Vector3d axis_dir = normalize_or_default(axis_seed, Eigen::Vector3d::UnitZ());
  const Frame frame = make_frame(axis_dir, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());

  std::vector<Eigen::Vector2d> projected;
  projected.reserve(points.size());
  for (const auto & p : points) {
    const Eigen::Vector3d diff = p - mean;
    projected.emplace_back(frame.x.dot(diff), frame.y.dot(diff));
  }

  const CircleFitResult circle = fit_circle_pratt(projected);
  if (!circle.success || !std::isfinite(circle.radius) || circle.radius <= 0.0) {
    return candidate;
  }

  const Eigen::Vector3d axis_point = mean + frame.x * circle.center.x() + frame.y * circle.center.y();
  std::vector<double> radial_distances;
  radial_distances.reserve(points.size());
  double t_min = std::numeric_limits<double>::infinity();
  double t_max = -std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    const Eigen::Vector3d diff = p - axis_point;
    const double t = diff.dot(axis_dir);
    t_min = std::min(t_min, t);
    t_max = std::max(t_max, t);
    const Eigen::Vector3d radial = diff - t * axis_dir;
    const double radial_dist = radial.norm();
    if (std::isfinite(radial_dist)) {
      radial_distances.push_back(radial_dist);
    }
  }

  if (!std::isfinite(t_min) || !std::isfinite(t_max) || radial_distances.size() < 3) {
    return candidate;
  }

  const double radius = median(radial_distances);
  const double length = t_max - t_min;
  if (!std::isfinite(radius) || radius <= 0.0 || !std::isfinite(length) || length < 1e-6) {
    return candidate;
  }

  double rmse_acc = 0.0;
  for (const double radial_dist : radial_distances) {
    const double residual = radial_dist - radius;
    rmse_acc += residual * residual;
  }

  candidate.valid = true;
  candidate.axis_dir = axis_dir;
  candidate.axis_point = axis_point;
  candidate.radius = radius;
  candidate.length = length;
  candidate.t_min = t_min;
  candidate.t_max = t_max;
  candidate.radial_rmse = std::sqrt(rmse_acc / static_cast<double>(radial_distances.size()));
  return candidate;
}
}  // namespace

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
  CylinderCandidate best_candidate;
  double best_score = std::numeric_limits<double>::infinity();
  for (int col = 0; col < 3; ++col) {
    const CylinderCandidate candidate = fit_cylinder_axis_candidate(points, mean, V.col(col));
    if (!candidate.valid) {
      continue;
    }

    const double score = candidate.radial_rmse / std::max(candidate.radius, 1e-9);
    if (score < best_score) {
      best_score = score;
      best_candidate = candidate;
    }
  }

  if (!best_candidate.valid) {
    return result;
  }

  Eigen::Vector3d axis_dir = best_candidate.axis_dir;
  Eigen::Vector3d min_point = best_candidate.axis_point + best_candidate.t_min * axis_dir;
  Eigen::Vector3d max_point = best_candidate.axis_point + best_candidate.t_max * axis_dir;

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
  result.radius = best_candidate.radius;
  result.length = length;
  result.top = top;
  result.bottom = bottom;
  result.axis = axis_vec;
  return result;
}

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

CircleFitResult fit_circle_ransac(
  const std::vector<Eigen::Vector2d> & points,
  const size_t iterations,
  const double inlier_tol,
  const size_t min_inliers,
  const uint32_t seed)
{
  CircleFitResult result;
  if (points.size() < 3) {
    return result;
  }

  std::mt19937 rng(seed);
  std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

  std::vector<size_t> best_inliers;
  CircleModel2d best_model;

  for (size_t iter = 0; iter < iterations; ++iter) {
    size_t i1 = dist(rng);
    size_t i2 = dist(rng);
    size_t i3 = dist(rng);
    if (i1 == i2 || i1 == i3 || i2 == i3) {
      continue;
    }

    const CircleModel2d model = circle_from_3_points(points[i1], points[i2], points[i3]);
    if (!model.valid) {
      continue;
    }

    std::vector<size_t> inliers;
    inliers.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      const double dist_to_center = (points[i] - model.center).norm();
      if (std::abs(dist_to_center - model.radius) <= inlier_tol) {
        inliers.push_back(i);
      }
    }

    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      best_model = model;
    }
  }

  if (best_inliers.size() < min_inliers) {
    return result;
  }

  std::vector<Eigen::Vector2d> inlier_points;
  inlier_points.reserve(best_inliers.size());
  for (const size_t idx : best_inliers) {
    inlier_points.push_back(points[idx]);
  }

  const CircleFitResult refined = fit_circle_pratt(inlier_points);
  if (refined.success) {
    return refined;
  }

  result.success = true;
  result.center = best_model.center;
  result.radius = best_model.radius;
  result.rmse = circle_rmse(inlier_points, best_model.center, best_model.radius);
  return result;
}

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

  double suu = 0.0;
  double suv = 0.0;
  double svv = 0.0;
  double suuu = 0.0;
  double suvv = 0.0;
  double svvv = 0.0;
  double svuu = 0.0;

  for (const auto & p : points) {
    const double u = p.x() - mean.x();
    const double v = p.y() - mean.y();
    const double uu = u * u;
    const double vv = v * v;

    suu += uu;
    svv += vv;
    suv += u * v;
    suuu += uu * u;
    svvv += vv * v;
    suvv += u * vv;
    svuu += v * uu;
  }

  const double det = 2.0 * (suu * svv - suv * suv);
  if (std::abs(det) < 1e-12) {
    return result;
  }

  const double uc = (svv * (suuu + suvv) - suv * (svvv + svuu)) / det;
  const double vc = (suu * (svvv + svuu) - suv * (suuu + suvv)) / det;

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

uint64_t edge_key(const uint32_t a, const uint32_t b)
{
  const uint32_t lo = std::min(a, b);
  const uint32_t hi = std::max(a, b);
  return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

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

}  // namespace hole_toolpath_planner::detail
