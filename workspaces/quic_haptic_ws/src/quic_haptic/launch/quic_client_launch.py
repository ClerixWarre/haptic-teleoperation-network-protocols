from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    server_address_arg = DeclareLaunchArgument(
        'server_address',
        default_value='localhost',
        description='QUIC server address'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='4567',
        description='QUIC server port'
    )
    
    client_name_arg = DeclareLaunchArgument(
        'client_name',
        default_value='quic_haptic_client',
        description='Name for this client instance'
    )

    # Create the client node
    quic_client_node = Node(
        package='quic_haptic',
        executable='quic_haptic_client',
        name=LaunchConfiguration('client_name'),
        output='screen',
        parameters=[{
            'server_address': LaunchConfiguration('server_address'),
            'server_port': LaunchConfiguration('server_port'),
        }]
    )

    return LaunchDescription([
        server_address_arg,
        server_port_arg,
        client_name_arg,
        quic_client_node
    ])
