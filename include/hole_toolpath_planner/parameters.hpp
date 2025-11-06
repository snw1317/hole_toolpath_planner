#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace hole_toolpath_planner
{
struct DetectionParameters
{
  std::string mode;
  double dedupe_angle_deg;
  double dedupe_center_tol;
  double dedupe_radius_tol;
};

struct SamplingParameters
{
  std::string strategy;
  int points;
  int seed;
};

struct NormalsParameters
{
  int k;
  std::array<double, 3> viewpoint;
};

struct SurfaceCircleParameters
{
  int min_loop_vertices;
  double circularity_rmse_thresh;
  std::string plane_fit_method;
  std::string outer_boundary_policy;
  std::array<double, 3> into_hint;
  double thickness;
  double min_radius;
  double max_radius;
  bool has_thickness;
};

struct CylinderFitParameters
{
  double distance_threshold;
  double normal_distance_weight;
  int max_iterations;
  int min_inliers;
  double min_inlier_ratio;
  double radius_min;
  double radius_max;
  double axis_alignment_min;
  double length_max;
  bool extract_iteratively;
};

struct PoseParameters
{
  std::string z_direction_rule;
  std::string x_seed;
  double neighbor_radius_scale;
  double neighbor_slab_thickness;
  std::string gram_schmidt_fallback;
};

struct ToolpathParameters
{
  bool generate;
  std::string strategy;
  double approach_offset;
  double stepdown;
  double feedrate;
  double spiral_pitch;
};

struct RvizParameters
{
  std::string marker_ns;
  double sphere_scale;
  double axis_length;
};

struct LoggingParameters
{
  std::string frame_id;
  std::string min_severity;
};

struct PlannerParameters
{
  DetectionParameters detection;
  SamplingParameters sampling;
  NormalsParameters normals;
  SurfaceCircleParameters surface_circle;
  CylinderFitParameters cylinder_fit;
  PoseParameters pose;
  ToolpathParameters toolpath;
  RvizParameters rviz;
  LoggingParameters logging;
};

PlannerParameters declare_and_get_parameters(rclcpp::Node & node);

}  // namespace hole_toolpath_planner
