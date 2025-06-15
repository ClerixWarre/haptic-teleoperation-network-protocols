import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='localhost',
        description='SCTP server hostname or IP address'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    )
    
    stats_interval_arg = DeclareLaunchArgument(
        'stats_interval',
        default_value='1000',
        description='Statistics reporting interval (messages)'
    )
    
    # Get launch configurations
    server_host = LaunchConfiguration('server_host')
    verbose = LaunchConfiguration('verbose')
    stats_interval = LaunchConfiguration('stats_interval')
    
    # Server node
    sctp_server_node = Node(
        package='sctp_haptic_teleoperation',
        executable='sctp_haptic_server',
        name='sctp_haptic_server',
        output='screen',
        arguments=[
            '--stats-interval=' + stats_interval,
            # Add verbose flag conditionally
        ],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # Client node (delayed start to ensure server is ready)
    sctp_client_node = Node(
        package='sctp_haptic_teleoperation',
        executable='sctp_haptic_client',
        name='sctp_haptic_client',
        output='screen',
        arguments=[
            server_host,
            '--stats-interval=' + stats_interval,
        ],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # Delay client start by 2 seconds
    delayed_client = TimerAction(
        period=2.0,
        actions=[sctp_client_node]
    )
    
    return LaunchDescription([
        server_host_arg,
        verbose_arg,
        stats_interval_arg,
        sctp_server_node,
        delayed_client,
    ])
