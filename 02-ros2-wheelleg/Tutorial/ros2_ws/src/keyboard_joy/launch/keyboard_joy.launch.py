from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Path to the key_mappings.yaml file'
        ),
        Node(
            package='keyboard_joy',
            executable='joy_node',
            name='keyboard_joy',
            output='screen',
            parameters=[{'config': LaunchConfiguration('config')}]
        )
    ])
