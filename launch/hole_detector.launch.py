from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():
    default_params = Path(__file__).resolve().parent.parent / "cfg" / "default_params.yaml"

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=str(default_params),
        description="YAML file with planner parameters"
    )

    node = Node(
        package="hole_toolpath_planner",
        executable="hole_detector_node",
        name="hole_toolpath_planner",
        output="screen",
        parameters=[LaunchConfiguration("params")]
    )

    return LaunchDescription([
        params_arg,
        node,
    ])
