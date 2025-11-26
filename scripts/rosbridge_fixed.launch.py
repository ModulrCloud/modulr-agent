from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port for rosbridge websocket server'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': '',
                'delay_between_messages': 0.0,  # DOUBLE
                'max_message_size': 10000000,  # INTEGER
                'fragment_timeout': 600.0,  # DOUBLE
                'unregister_timeout': 10.0,  # DOUBLE
                'use_compression': False,  # BOOL
                'topics_glob': '',
                'services_glob': '',
                'params_glob': '',
                'bson_only_mode': False,  # BOOL
                'websocket_ping_interval': 0,  # INTEGER (not DOUBLE!)
                'websocket_ping_timeout': 5.0,  # DOUBLE
            }]
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
        ),
    ])

