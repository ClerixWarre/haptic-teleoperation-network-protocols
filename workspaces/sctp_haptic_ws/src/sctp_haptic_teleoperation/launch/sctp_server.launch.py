from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
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
    
    processing_delay_arg = DeclareLaunchArgument(
        'processing_delay',
        default_value='0.0',
        description='Artificial processing delay in ms (for testing)'
    )
    
    # Get launch configurations
    verbose = LaunchConfiguration('verbose')
    stats_interval = LaunchConfiguration('stats_interval')
    processing_delay = LaunchConfiguration('processing_delay')
    
    # Build arguments list
    server_args = ['--stats-interval=' + stats_interval]
    
    # Server node
    sctp_server_node = Node(
        package='sctp_haptic_teleoperation',
        executable='sctp_haptic_server',
        name='sctp_haptic_server_surgical',
        output='screen',
        arguments=server_args,
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            ('/phantom/remote_state', '/phantom/remote_state'),
        ]
    )
    
    return LaunchDescription([
        verbose_arg,
        stats_interval_arg,
        processing_delay_arg,
        sctp_server_node,
    ])
