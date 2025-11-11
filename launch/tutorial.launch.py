from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publishing_flag',
            default_value='true',
            description='Flag to enable publishing'
        ),
        Node(
            package='beginner_tutorials',
            executable='talker',
            parameters=[{'publishing_flag': LaunchConfiguration('publishing_flag')}]
        )
    ])