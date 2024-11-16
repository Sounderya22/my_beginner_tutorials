"""
MIT License

Copyright (c) [year] [your name or organization]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

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
