from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("vesc_driver"),
                "launch",
                "vesc_driver_node.launch.py",
            ])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("urg_node2"),
                "launch",
                "urg_node2.launch.py",
            ])
        )
    )

    vehicle_control_node = Node(
        package="locomotion",
        executable="vehicle_control_node",
        name="vehicle_control_node",
        output="screen",
    )

    return LaunchDescription([
        vesc_launch,
        lidar_launch,
        vehicle_control_node,
    ])