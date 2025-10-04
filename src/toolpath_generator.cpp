#include "hole_toolpath_planner/toolpath_generator.hpp"

#include <cmath>

#include <Eigen/Core>

namespace hole_toolpath_planner
{
namespace
{
geometry_msgs::msg::PoseStamped make_pose(
  const geometry_msgs::msg::Pose & base_pose,
  const Eigen::Vector3d & offset,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose = base_pose;
  pose.pose.position.x += offset.x();
  pose.pose.position.y += offset.y();
  pose.pose.position.z += offset.z();
  return pose;
}
}  // namespace

std::vector<msg::Toolpath> build_toolpaths(
  const msg::HoleArray & holes,
  const ToolpathParameters & params,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  std::vector<msg::Toolpath> toolpaths;
  toolpaths.reserve(holes.holes.size());

  if (!params.generate) {
    return toolpaths;
  }

  for (const auto & hole : holes.holes) {
    msg::Toolpath tp;
    tp.header.stamp = stamp;
    tp.header.frame_id = frame_id;
    tp.hole_id = hole.id;
    tp.strategy = params.strategy;
    tp.stepdown = static_cast<float>(params.stepdown);
    tp.feedrate = static_cast<float>(params.feedrate);

    const Eigen::Vector3d axis{
      hole.axis.x,
      hole.axis.y,
      hole.axis.z};

    Eigen::Vector3d axis_unit = axis;
    const double axis_norm = axis.norm();
    if (axis_norm > 1e-9) {
      axis_unit /= axis_norm;
    } else {
      axis_unit = Eigen::Vector3d::UnitZ();
    }

    const Eigen::Vector3d origin{
      hole.pose.position.x,
      hole.pose.position.y,
      hole.pose.position.z};

    if (params.approach_offset > 0.0) {
      const Eigen::Vector3d offset = -params.approach_offset * axis_unit;
      tp.poses.push_back(make_pose(hole.pose, offset, stamp, frame_id));
    }

    tp.poses.push_back(make_pose(hole.pose, Eigen::Vector3d::Zero(), stamp, frame_id));

    const double length = static_cast<double>(hole.length);
    if (length > 0.0 && params.stepdown > 0.0) {
      const int steps = std::max(1, static_cast<int>(std::ceil(length / params.stepdown)));
      for (int i = 1; i <= steps; ++i) {
        const double depth = std::min(length, i * params.stepdown);
        const Eigen::Vector3d offset = axis_unit * depth;
        tp.poses.push_back(make_pose(hole.pose, offset, stamp, frame_id));
      }
    }

    toolpaths.push_back(std::move(tp));
  }

  return toolpaths;
}

}  // namespace hole_toolpath_planner
