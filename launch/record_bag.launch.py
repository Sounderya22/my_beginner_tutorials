from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime

def generate_launch_description():
    # Define the directory to store the bag files
    bag_directory = os.path.join(os.getcwd(), 'results')

    # Get the current timestamp in a desired format
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    # Declare a launch argument for enabling/disabling bag recording
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Flag to enable or disable bag recording'
    )

    # Function to conditionally include the bag recording process
    def conditionally_add_bag_recording(context):
        enable_recording = LaunchConfiguration('enable_recording').perform(context)
        if enable_recording.lower() == 'true':
            return [
                ExecuteProcess(
                    cmd=['mkdir', '-p', bag_directory],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(bag_directory, f'recording_{timestamp}')],
                    output='screen'
                ),
            ]
        return []

    return LaunchDescription([
        enable_recording_arg,
        OpaqueFunction(function=conditionally_add_bag_recording),
    ])