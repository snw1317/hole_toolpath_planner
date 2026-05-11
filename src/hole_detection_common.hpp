#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hole_toolpath_planner::detail
{
struct CylinderFitSimple
{
  bool valid{false};
  double radius{0.0};
  double length{0.0};
  Eigen::Vector3d top{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bottom{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
};

struct Frame
{
  Eigen::Vector3d x;
  Eigen::Vector3d y;
  Eigen::Vector3d z;
};

struct CircleFitResult
{
  bool success{false};
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double rmse{std::numeric_limits<double>::infinity()};
};

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

rclcpp::Logger::Level level_from_string(const std::string & value);

double to_radians(double degrees);

double axis_angle(const geometry_msgs::msg::Vector3 & a, const geometry_msgs::msg::Vector3 & b);

double distance_between(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

Eigen::Vector3d normalize_or_default(const Eigen::Vector3d & v, const Eigen::Vector3d & fallback);

CylinderFitSimple fit_cylinder_from_points(const std::vector<Eigen::Vector3d> & points);

Frame make_frame(
  const Eigen::Vector3d & z_dir,
  const Eigen::Vector3d & primary_seed,
  const Eigen::Vector3d & fallback_seed);

geometry_msgs::msg::Quaternion quaternion_from_frame(const Frame & frame);

CircleFitResult fit_circle_pratt(const std::vector<Eigen::Vector2d> & points);

CircleFitResult fit_circle_ransac(
  const std::vector<Eigen::Vector2d> & points,
  size_t iterations,
  double inlier_tol,
  size_t min_inliers,
  uint32_t seed);

double polygon_area_2d(const std::vector<Eigen::Vector2d> & points);

uint64_t edge_key(uint32_t a, uint32_t b);

Eigen::Vector3d select_seed(
  const std::string & key,
  const Eigen::Vector3d & major_axis,
  const Eigen::Vector3d & minor_axis);

bool fit_plane_pca(
  const std::vector<Eigen::Vector3d> & points,
  Eigen::Vector3d & centroid,
  Eigen::Vector3d & normal);

}  // namespace hole_toolpath_planner::detail
