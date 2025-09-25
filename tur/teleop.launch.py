from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # จอยสติ๊กไดรเวอร์
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 30.0
            }]
        ),
        # turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        # แปลง Joy → Twist
        Node(
            package='gp650_turtlesim_teleop',
            executable='teleop',
            name='gp650_turtlesim_teleop',
            parameters=['config/gp650_params.yaml']
        ),
    ])

