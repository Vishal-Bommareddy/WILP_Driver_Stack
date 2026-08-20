import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_config = os.path.join(
        get_package_share_directory('vehicle_config'),
        'config',
        'vehicle.yaml',
    )

    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vesc_driver'),
                'launch',
                'vesc_driver_node.launch.py',
            ])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('urg_node2'),
                'launch',
                'urg_node2.launch.py',
            ])
        )
    )

    return LaunchDescription([
        vesc_launch,
        lidar_launch,
        Node(
            package='locomotion',
            executable='vehicle_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[vehicle_config],
        ),
        Node(
            package='odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[vehicle_config],
        ),
        Node(
            package='safety',
            executable='safety_node',
            name='safety_node',
            output='screen',
        ),
    ])
