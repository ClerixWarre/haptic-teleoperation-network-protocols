from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='4567',
        description='Port for QUIC server'
    )
    
    cert_file_arg = DeclareLaunchArgument(
        'cert_file',
        default_value='server.cert',
        description='Path to server certificate file'
    )
    
    key_file_arg = DeclareLaunchArgument(
        'key_file',
        default_value='server.key',
        description='Path to server key file'
    )
    
    client_delay_arg = DeclareLaunchArgument(
        'client_delay',
        default_value='2.0',
        description='Delay in seconds before starting client'
    )

    # Create the server node
    quic_server_node = Node(
        package='quic_haptic',
        executable='quic_haptic_server',
        name='quic_haptic_server',
        output='screen',
        parameters=[{
            'server_port': LaunchConfiguration('server_port'),
            'cert_file': LaunchConfiguration('cert_file'),
            'key_file': LaunchConfiguration('key_file'),
        }]
    )

    # Create the client node with a delay to ensure server is ready
    quic_client_node = TimerAction(
        period=LaunchConfiguration('client_delay'),
        actions=[
            Node(
                package='quic_haptic',
                executable='quic_haptic_client',
                name='quic_haptic_client',
                output='screen',
                parameters=[{
                    'server_address': 'localhost',
                    'server_port': LaunchConfiguration('server_port'),
                }]
            )
        ]
    )

    return LaunchDescription([
        server_port_arg,
        cert_file_arg,
        key_file_arg,
        client_delay_arg,
        quic_server_node,
        quic_client_node
    ])
