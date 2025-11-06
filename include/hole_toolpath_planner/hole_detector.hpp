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
  struct SurfaceDiagnostics
  {
    bool attempted{false};
    bool mesh_empty{false};
    bool vertices_empty{false};
    bool boundary_edges_empty{false};
    size_t face_count{0};
    size_t boundary_edge_count{0};
    size_t loops_total{0};
    size_t loops_invalid_topology{0};
    size_t loops_too_small{0};
    size_t loops_invalid_vertices{0};
    size_t loops_considered{0};
    size_t loops_outer_skipped{0};
    size_t circle_fit_failures{0};
    size_t circle_fit_success{0};
    size_t rmse_rejections{0};
    size_t radius_rejections{0};
    size_t detections_emitted{0};
  };

  struct SolidDiagnostics
  {
    bool attempted{false};
    bool missing_vertices{false};
    bool missing_polygons{false};
    bool no_valid_triangles{false};
    bool area_too_small{false};
    bool insufficient_samples{false};
    bool invalid_radius_constraints{false};
    size_t vertex_count{0};
    size_t polygon_count{0};
    size_t valid_triangles{0};
    size_t samples_requested{0};
    size_t samples_generated{0};
    size_t segmentation_attempts{0};
    size_t cylinders_emitted{0};
    size_t cylinders_rejected_length{0};
    size_t cylinders_rejected_alignment{0};
    size_t cylinders_invalid_origin{0};
    size_t cylinders_no_entry_plane{0};
    size_t cylinders_rejected_inliers{0};
    size_t min_inliers_required{0};
    double min_inlier_ratio{0.0};
    size_t last_inlier_count{0};
    double last_inlier_ratio{0.0};
  };

  bool load_mesh(const std::string & mesh_path, pcl::PolygonMesh & mesh) const;

  std::vector<msg::Hole> detect_surface_mode_legacy(
    const pcl::PolygonMesh & mesh,
    const srv::DetectHoles::Request & request,
    SurfaceDiagnostics * diagnostics = nullptr) const;

  std::vector<msg::Hole> detect_surface_clusters(
    const pcl::PolygonMesh & mesh,
    const srv::DetectHoles::Request & request,
    SurfaceDiagnostics * diagnostics = nullptr) const;

  std::vector<msg::Hole> detect_solid_mode(
    const pcl::PolygonMesh & mesh,
    const srv::DetectHoles::Request & request,
    SolidDiagnostics * diagnostics = nullptr) const;

  std::vector<msg::Hole> deduplicate(std::vector<msg::Hole> && holes) const;

  msg::HoleArray assemble_response(
    std::vector<msg::Hole> && holes,
    const rclcpp::Time & stamp) const;

  rclcpp::Node & node_;
  PlannerParameters params_;
  rclcpp::Logger logger_;
};

}  // namespace hole_toolpath_planner
