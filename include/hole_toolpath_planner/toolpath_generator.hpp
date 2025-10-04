#pragma once

#include <vector>

#include "hole_toolpath_planner/msg/hole_array.hpp"
#include "hole_toolpath_planner/msg/toolpath.hpp"
#include "hole_toolpath_planner/parameters.hpp"

#include <rclcpp/time.hpp>

namespace hole_toolpath_planner
{
std::vector<msg::Toolpath> build_toolpaths(
  const msg::HoleArray & holes,
  const ToolpathParameters & params,
  const rclcpp::Time & stamp,
  const std::string & frame_id);

}  // namespace hole_toolpath_planner
