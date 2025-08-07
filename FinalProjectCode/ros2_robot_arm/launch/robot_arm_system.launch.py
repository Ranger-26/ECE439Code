from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_file = 'ros2_robot_arm/params.yaml'
    return LaunchDescription([
        Node(
            package='ros2_robot_arm',
            executable='stereo_vision_position_detection',
            name='stereo_vision_position_detection',
            output='screen',
            parameters=[param_file]
        ),
        Node(
            package='ros2_robot_arm',
            executable='arm_position_setter',
            name='arm_position_setter',
            output='screen'
        ),
        Node(
            package='ros2_robot_arm',
            executable='inverse_kinematics',
            name='inverse_kinematics',
            output='screen',
            parameters=[param_file]
        ),
        Node(
            package='ros2_robot_arm',
            executable='servo_commands',
            name='servo_commands',
            output='screen',
            parameters=[param_file]
        ),
        # Uncomment below to enable manual override GUI
        # Node(
        #     package='ros2_robot_arm',
        #     executable='servo_gui',
        #     name='servo_gui',
        #     output='screen'
        # ),
    ])
