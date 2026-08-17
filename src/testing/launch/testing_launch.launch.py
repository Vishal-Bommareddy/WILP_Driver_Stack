from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vesc_driver'),
                'launch',
                'vesc_driver_node.launch.py',
            ])))

    urg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('urg_node2'),
                'launch',
                'urg_node2.launch.py',
            ])))

    return LaunchDescription([
        vesc_launch,
        urg_launch,
        Node(
            package='testing',
            executable='vehicle_control_node',
            name='testing_vehicle_control_node',
            output='screen',
        ),
    ])
