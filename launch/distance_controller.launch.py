from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    distance_controller_node = Node(
        package='distance_controller',
        executable='distance_controller',
        name='distance_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['1']
    )

    return LaunchDescription([
        distance_controller_node
    ])
