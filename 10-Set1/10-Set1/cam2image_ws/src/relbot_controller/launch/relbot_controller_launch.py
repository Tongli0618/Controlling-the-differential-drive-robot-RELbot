from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools_sdfr',
            executable='cam2image',
            name='cam2image',
            parameters=['cam2image.yaml'],
            arguments=['--ros-args', '-p', 'depth:=1', '-p', 'history:=keep_last'],
        ),  
        #Node(
        #    package='lpindicator',
        #    executable='lp_indicator_node',
        #    name='lp_indicator_node',
        #),
        Node(
            package='image_tools',
            executable='showimage',
            name='showimage',
            remappings=[('image', '/processed_image_topic')],
        ),
        Node(
            package='RELbot_simulator',
            executable='RELbot_simulator',
            name='RELbot_simulator',
        ),
        Node(
            package='relbot_controller',
            executable='wheel_control_node',
            name='wheel_control_node',
        #   parameters=[{'initial_left_motor_vel': 0.0, 'initial_right_motor_vel': 0.0}],
        ),
    ])
