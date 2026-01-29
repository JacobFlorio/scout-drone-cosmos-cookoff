from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    px4_heartbeat = Node(
        package='asd_px4',
        executable='px4_heartbeat_node',
        name='px4_heartbeat_node',
        output='screen',
        parameters=[{
            'topic': 'asd/px4/heartbeat',
            'rate_hz': 2.0
        }]
    )

    monitor = Node(
        package='asd_safety',
        executable='heartbeat_monitor_node',
        name='heartbeat_monitor_node',
        output='screen',
        parameters=[{
            'required_heartbeats': [
                'asd/px4/heartbeat',
            ],
            'timeout_sec': 2.0,
            'publish_rate_hz': 2.0
        }]
    )

    return LaunchDescription([px4_heartbeat, monitor])
