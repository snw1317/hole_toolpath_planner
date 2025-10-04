#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/PolygonMesh.h>

#include "hole_toolpath_planner/msg/hole_array.hpp"
#include "hole_toolpath_planner/msg/toolpath.hpp"
#include "hole_toolpath_planner/srv/detect_holes.hpp"
#include "hole_toolpath_planner/parameters.hpp"

namespace hole_toolpath_planner
{
class HoleDetector
{
public:
  HoleDetector(rclcpp::Node & node, PlannerParameters params);

  msg::HoleArray detect(const srv::DetectHoles::Request & request);

  std::vector<msg::Toolpath> make_toolpaths(const msg::HoleArray & holes, const rclcpp::Time & stamp);

  const PlannerParameters & parameters() const {return params_;}

private:
  bool load_mesh(const std::string & mesh_path, pcl::PolygonMesh & mesh) const;

  std::vector<msg::Hole> detect_surface_mode(
    const pcl::PolygonMesh & mesh,
    const srv::DetectHoles::Request & request) const;

  std::vector<msg::Hole> detect_solid_mode(
    const pcl::PolygonMesh & mesh,
    const srv::DetectHoles::Request & request) const;

  std::vector<msg::Hole> deduplicate(std::vector<msg::Hole> && holes) const;

  msg::HoleArray assemble_response(
    std::vector<msg::Hole> && holes,
    const rclcpp::Time & stamp) const;

  rclcpp::Node & node_;
  PlannerParameters params_;
  rclcpp::Logger logger_;
};

}  // namespace hole_toolpath_planner
