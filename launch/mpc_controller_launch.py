from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #
        Node(
            package='mpc_controller_pkg',
            executable='mpc_controller',
            name='mpc_controller_node',
            output='screen'
        ),
        # 
        Node(
            package='mpc_controller_pkg',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen'
        ),
    ])
