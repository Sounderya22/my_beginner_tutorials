from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for publish frequency
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',  # default frequency (in Hz)
        description='Frequency (in Hz) for the Talker node to publish messages'
    )

    # Use the LaunchConfiguration to retrieve the command-line argument
    publish_frequency = LaunchConfiguration('publish_frequency')

    # Define the client node (launch first)
    client_node = Node(
        package='beginner_tutorials',
        executable='client',
        name='client',
        parameters=[{'publish_frequency': publish_frequency}]
    )

    # Define the server node with a delay (launch second)
    server_node = TimerAction(
        period=5.0,  # delay in seconds before launching the server
        actions=[
            Node(
                package='beginner_tutorials',
                executable='server',
                name='server'
            )
        ]
    )

    # Launch description with both nodes and the argument
    return LaunchDescription([
        publish_frequency_arg,
        client_node,
        server_node
    ])
