#include "hole_toolpath_planner/parameters.hpp"

#include <algorithm>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace hole_toolpath_planner
{
namespace
{
std::array<double, 3> to_array3(const std::vector<double> & values, const std::array<double, 3> & fallback)
{
  std::array<double, 3> result = fallback;
  for (size_t i = 0; i < std::min(values.size(), static_cast<size_t>(3)); ++i) {
    result[i] = values[i];
  }
  return result;
}
}  // namespace

PlannerParameters declare_and_get_parameters(rclcpp::Node & node)
{
  PlannerParameters params{};

  params.detection.mode = node.declare_parameter<std::string>("detection.mode", "auto");
  params.detection.dedupe_angle_deg = node.declare_parameter<double>("detection.dedupe_angle_deg", 5.0);
  params.detection.dedupe_center_tol = node.declare_parameter<double>("detection.dedupe_center_tol", 0.0005);
  params.detection.dedupe_radius_tol = node.declare_parameter<double>("detection.dedupe_radius_tol", 0.0005);

  params.sampling.strategy = node.declare_parameter<std::string>("sampling.strategy", "uniform_area");
  params.sampling.points = node.declare_parameter<int>("sampling.points", 150000);
  params.sampling.seed = node.declare_parameter<int>("sampling.seed", 42);

  params.normals.k = node.declare_parameter<int>("normals.k", 30);
  auto viewpoint_vec = node.declare_parameter<std::vector<double>>(
    "normals.viewpoint", std::vector<double>{0.0, 0.0, 0.0});
  params.normals.viewpoint = to_array3(viewpoint_vec, {0.0, 0.0, 0.0});

  params.surface_circle.min_loop_vertices = node.declare_parameter<int>("surface_circle.min_loop_vertices", 6);
  params.surface_circle.circularity_rmse_thresh = node.declare_parameter<double>(
    "surface_circle.circularity_rmse_thresh", 0.002);
  params.surface_circle.plane_fit_method = node.declare_parameter<std::string>(
    "surface_circle.plane_fit_method", "pca");
  params.surface_circle.outer_boundary_policy = node.declare_parameter<std::string>(
    "surface_circle.outer_boundary_policy", "largest_by_area");
  auto into_hint_vec = node.declare_parameter<std::vector<double>>(
    "surface_circle.into_hint", std::vector<double>{0.0, 0.0, 1.0});
  params.surface_circle.into_hint = to_array3(into_hint_vec, {0.0, 0.0, 1.0});

  rcl_interfaces::msg::ParameterDescriptor thickness_descriptor;
  thickness_descriptor.name = "surface_circle.thickness";
  thickness_descriptor.dynamic_typing = true;  // allow unset/null in parameter files
  thickness_descriptor.description =
    "Optional known thickness in meters; leave unset or set to null to disable.";
  node.declare_parameter(
    "surface_circle.thickness", rclcpp::ParameterValue{}, thickness_descriptor);

  const auto thickness_param = node.get_parameter("surface_circle.thickness");
  const auto thickness_type = thickness_param.get_type();
  params.surface_circle.has_thickness = thickness_type == rclcpp::ParameterType::PARAMETER_DOUBLE;
  if (params.surface_circle.has_thickness) {
    params.surface_circle.thickness = thickness_param.as_double();
  } else {
    if (thickness_type != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_WARN(
        node.get_logger(),
        "Ignoring parameter 'surface_circle.thickness' with unsupported type (%d); expected double.",
        static_cast<int>(thickness_type));
    }
    params.surface_circle.thickness = 0.0;
  }
  params.surface_circle.min_radius = node.declare_parameter<double>("surface_circle.min_radius", 0.001);
  params.surface_circle.max_radius = node.declare_parameter<double>("surface_circle.max_radius", 0.050);

  params.cylinder_fit.distance_threshold = node.declare_parameter<double>(
    "cylinder_fit.distance_threshold", 0.0008);
  params.cylinder_fit.normal_distance_weight = node.declare_parameter<double>(
    "cylinder_fit.normal_distance_weight", 0.1);
  params.cylinder_fit.max_iterations = node.declare_parameter<int>(
    "cylinder_fit.max_iterations", 10000);
  params.cylinder_fit.min_inliers = node.declare_parameter<int>(
    "cylinder_fit.min_inliers", 800);
  params.cylinder_fit.radius_min = node.declare_parameter<double>(
    "cylinder_fit.radius_min", 0.001);
  params.cylinder_fit.radius_max = node.declare_parameter<double>(
    "cylinder_fit.radius_max", 0.050);
  params.cylinder_fit.extract_iteratively = node.declare_parameter<bool>(
    "cylinder_fit.extract_iteratively", true);

  params.pose.z_direction_rule = node.declare_parameter<std::string>("pose.z_direction_rule", "into_part");
  params.pose.x_seed = node.declare_parameter<std::string>("pose.x_seed", "world_x");
  params.pose.neighbor_radius_scale = node.declare_parameter<double>("pose.neighbor_radius_scale", 2.0);
  params.pose.neighbor_slab_thickness = node.declare_parameter<double>(
    "pose.neighbor_slab_thickness", 0.002);
  params.pose.gram_schmidt_fallback = node.declare_parameter<std::string>(
    "pose.gram_schmidt_fallback", "mesh_x");

  params.toolpath.generate = node.declare_parameter<bool>("toolpath.generate", false);
  params.toolpath.strategy = node.declare_parameter<std::string>("toolpath.strategy", "circle");
  params.toolpath.approach_offset = node.declare_parameter<double>("toolpath.approach_offset", 0.010);
  params.toolpath.stepdown = node.declare_parameter<double>("toolpath.stepdown", 0.002);
  params.toolpath.feedrate = node.declare_parameter<double>("toolpath.feedrate", 0.050);
  params.toolpath.spiral_pitch = node.declare_parameter<double>("toolpath.spiral_pitch", 0.005);

  params.rviz.marker_ns = node.declare_parameter<std::string>("rviz.marker_ns", "holes");
  params.rviz.sphere_scale = node.declare_parameter<double>("rviz.sphere_scale", 0.003);
  params.rviz.axis_length = node.declare_parameter<double>("rviz.axis_length", 0.020);

  params.logging.frame_id = node.declare_parameter<std::string>("logging.frame_id", "world");
  params.logging.min_severity = node.declare_parameter<std::string>("logging.min_severity", "info");

  return params;
}

}  // namespace hole_toolpath_planner
