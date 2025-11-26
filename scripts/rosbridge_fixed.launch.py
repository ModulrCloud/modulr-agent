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
                # Only set essential parameters, let others use defaults
                # This avoids parameter type conflicts in rosbridge Jazzy
            }]
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
        ),
    ])

