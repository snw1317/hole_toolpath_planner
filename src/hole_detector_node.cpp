#include "hole_toolpath_planner/hole_detector.hpp"

#include <Eigen/Core>
#include <array>
#include <memory>
#include <sstream>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace hole_toolpath_planner
{
namespace msg = ::hole_toolpath_planner::msg;
namespace srv = ::hole_toolpath_planner::srv;
namespace
{
std::array<float, 4> rgba(float r, float g, float b, float a)
{
  return {r, g, b, a};
}
}  // namespace

class HoleToolpathPlannerNode : public rclcpp::Node
{
public:
  explicit HoleToolpathPlannerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("hole_toolpath_planner", options),
    params_(declare_and_get_parameters(*this)),
    detector_(*this, params_)
  {
    hole_pub_ = create_publisher<msg::HoleArray>("holes", rclcpp::QoS{10}.reliable());
    toolpath_pub_ = create_publisher<msg::Toolpath>("toolpaths", rclcpp::QoS{10}.reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "hole_markers", rclcpp::QoS{10}.transient_local());

    using std::placeholders::_1;
    using std::placeholders::_2;
    service_ = create_service<srv::DetectHoles>(
      "detect_holes",
      std::bind(&HoleToolpathPlannerNode::detect_callback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "hole_toolpath_planner ready");
  }

private:
  void detect_callback(
    const std::shared_ptr<srv::DetectHoles::Request> request,
    std::shared_ptr<srv::DetectHoles::Response> response)
  {
    const auto holes = detector_.detect(*request);
    response->holes = holes;

    hole_pub_->publish(holes);

    const rclcpp::Time stamp(holes.header.stamp);
    auto toolpaths = detector_.make_toolpaths(holes, stamp);
    for (auto & tp : toolpaths) {
      toolpath_pub_->publish(tp);
    }

    auto markers = make_markers(holes, stamp);
    if (!markers.markers.empty()) {
      marker_pub_->publish(markers);
    }
  }

  visualization_msgs::msg::MarkerArray make_markers(
    const msg::HoleArray & holes,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    int32_t base_id = 0;

    for (const auto & hole : holes.holes) {
      const auto frame_id = params_.logging.frame_id;
      const Eigen::Vector3d axis{hole.axis.x, hole.axis.y, hole.axis.z};
      Eigen::Vector3d axis_unit = axis.norm() > 1e-9 ? axis.normalized() : Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d origin{hole.pose.position.x, hole.pose.position.y, hole.pose.position.z};

      visualization_msgs::msg::Marker sphere;
      sphere.header.stamp = stamp;
      sphere.header.frame_id = frame_id;
      sphere.ns = params_.rviz.marker_ns;
      sphere.id = base_id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = hole.pose;
      sphere.scale.x = params_.rviz.sphere_scale;
      sphere.scale.y = params_.rviz.sphere_scale;
      sphere.scale.z = params_.rviz.sphere_scale;
      const auto sphere_color = rgba(0.1f, 0.7f, 0.3f, 0.8f);
      sphere.color.r = sphere_color[0];
      sphere.color.g = sphere_color[1];
      sphere.color.b = sphere_color[2];
      sphere.color.a = sphere_color[3];
      array.markers.push_back(sphere);

      visualization_msgs::msg::Marker axis_marker;
      axis_marker.header.stamp = stamp;
      axis_marker.header.frame_id = frame_id;
      axis_marker.ns = params_.rviz.marker_ns + "_axis";
      axis_marker.id = base_id++;
      axis_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      axis_marker.action = visualization_msgs::msg::Marker::ADD;
      axis_marker.scale.x = params_.rviz.sphere_scale * 0.2;
      const auto axis_color = rgba(0.2f, 0.4f, 0.9f, 0.9f);
      axis_marker.color.r = axis_color[0];
      axis_marker.color.g = axis_color[1];
      axis_marker.color.b = axis_color[2];
      axis_marker.color.a = axis_color[3];
      geometry_msgs::msg::Point p0;
      p0.x = origin.x() - axis_unit.x() * params_.rviz.axis_length;
      p0.y = origin.y() - axis_unit.y() * params_.rviz.axis_length;
      p0.z = origin.z() - axis_unit.z() * params_.rviz.axis_length;
      geometry_msgs::msg::Point p1;
      p1.x = origin.x() + axis_unit.x() * params_.rviz.axis_length;
      p1.y = origin.y() + axis_unit.y() * params_.rviz.axis_length;
      p1.z = origin.z() + axis_unit.z() * params_.rviz.axis_length;
      axis_marker.points.push_back(p0);
      axis_marker.points.push_back(p1);
      array.markers.push_back(axis_marker);

      visualization_msgs::msg::Marker text;
      text.header.stamp = stamp;
      text.header.frame_id = frame_id;
      text.ns = params_.rviz.marker_ns + "_label";
      text.id = base_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = hole.pose;
      text.pose.position.z += params_.rviz.sphere_scale * 1.5;
      text.scale.z = params_.rviz.sphere_scale;
      const auto text_color = rgba(1.0f, 1.0f, 1.0f, 0.9f);
      text.color.r = text_color[0];
      text.color.g = text_color[1];
      text.color.b = text_color[2];
      text.color.a = text_color[3];
      std::ostringstream oss;
      oss.setf(std::ios::fixed, std::ios::floatfield);
      oss.precision(3);
      oss << "id=" << hole.id << " ⌀=" << hole.diameter * 1000.0 << " mm";
      text.text = oss.str();
      array.markers.push_back(text);
    }

    return array;
  }

  PlannerParameters params_;
  HoleDetector detector_;
  rclcpp::Service<srv::DetectHoles>::SharedPtr service_;
  rclcpp::Publisher<msg::HoleArray>::SharedPtr hole_pub_;
  rclcpp::Publisher<msg::Toolpath>::SharedPtr toolpath_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace hole_toolpath_planner

RCLCPP_COMPONENTS_REGISTER_NODE(hole_toolpath_planner::HoleToolpathPlannerNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<hole_toolpath_planner::HoleToolpathPlannerNode>(rclcpp::NodeOptions{});
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
