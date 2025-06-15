from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1000',
        description='Initial message rate in Hz'
    )
    
    # Get launch configurations
    server_host = LaunchConfiguration('server_host')
    verbose = LaunchConfiguration('verbose')
    stats_interval = LaunchConfiguration('stats_interval')
    rate = LaunchConfiguration('rate')
    
    # Build arguments list
    client_args = [
        server_host,
        '--stats-interval=' + stats_interval,
        '--rate=' + rate,
    ]
    
    # Client node
    sctp_client_node = Node(
        package='sctp_haptic_teleoperation',
        executable='sctp_haptic_client',
        name='sctp_haptic_client',
        output='screen',
        arguments=client_args,
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            ('/phantom/state', '/phantom/state'),
        ]
    )
    
    return LaunchDescription([
        server_host_arg,
        verbose_arg,
        stats_interval_arg,
        rate_arg,
        sctp_client_node,
    ])
