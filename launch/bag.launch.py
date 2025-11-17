from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch argument to enable/disable bag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable or disable ros2 bag recording.'
    )

    # ros2 bag record process (records all topics)
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    return LaunchDescription([
        record_bag_arg,
        rosbag_record
    ])
