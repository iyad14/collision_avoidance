import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    collision_avoidance = Node(
        package='collision_avoidance',
        executable='collision_avoidance',
        name='collision_avoidance',
        output='screen',
    )

    # Define the rosbag play command
    rosbag_file = LaunchConfiguration('rosbag_file', default='src/collision_avoidance/data/train1')
    qos_override_config_path = 'src/achilles/config/qos_policies.yaml'
    
    rosbag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--qos-profile-overrides-path', qos_override_config_path , rosbag_file],
        output='screen'
    )

    return LaunchDescription([
        rosbag_player,
        collision_avoidance
    ])

