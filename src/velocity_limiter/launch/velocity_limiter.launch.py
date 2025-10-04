##velocity_limiter.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('velocity_limiter')
    params = os.path.join(pkg_share, 'params', 'velocity_limiter.yaml')

    return LaunchDescription([
        Node(
            package='velocity_limiter',
            executable='velocity_limiter_node',
            name='velocity_limiter',
            parameters=[params],
            remappings=[
                ('/cmd_avoid', '/planner/cmd_avoid'),
                ('/cmd_vel_policy', '/cmd_policy'),
            ],
            output='screen'
        )
    ])

