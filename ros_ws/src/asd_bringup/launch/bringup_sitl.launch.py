#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    # Vehicle model for PX4 gz sim (your script uses gz_x500)
    vehicle = LaunchConfiguration('vehicle', default='gz_x500')

    # World file: either "empty" (uses your repo world) or an absolute path
    world = LaunchConfiguration('world', default=str(
        Path.home() / 'autonomous-security-drone' / 'sim' / 'px4_gz' / 'worlds' / 'empty.world'
    ))

    # PX4-Autopilot submodule path inside your repo
    px4_dir = str(Path.home() / 'autonomous-security-drone' / 'sim' / 'px4' / 'PX4-Autopilot')

    # Micro XRCE-DDS Agent port
    agent_port = LaunchConfiguration('agent_port', default='8888')

    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='gz_x500', description='PX4 SITL vehicle target (e.g. gz_x500)'),
        DeclareLaunchArgument('world', default_value=str(Path.home() / 'autonomous-security-drone' / 'sim' / 'px4_gz' / 'worlds' / 'empty.world'),
                              description='World file path (absolute)'),
        DeclareLaunchArgument('agent_port', default_value='8888', description='Micro XRCE-DDS Agent UDP port'),

        # Start Micro XRCE-DDS Agent (PX4 uxrce_dds_client connects to UDP 8888 by default)
        ExecuteProcess(
            cmd=['bash', '-lc', 'MicroXRCEAgent udp4 -p ' + str(agent_port)],
            output='screen',
            name='micro_xrce_agent'
        ),

        # Start PX4 SITL (Gazebo/gz target)
        ExecuteProcess(
            cmd=['bash', '-lc', f'cd "{px4_dir}" && make px4_sitl {vehicle}'],
            output='screen',
            name='px4_sitl'
        ),
    ])
