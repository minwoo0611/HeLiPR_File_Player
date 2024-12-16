import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node  # <-- Correct import for Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('driver', default_value='helipr_file_player', description='Driver package'),
        DeclareLaunchArgument('output', default_value='screen', description='Output type'),
        
        # Driver node
        Node(
            package=LaunchConfiguration('driver'),
            executable=LaunchConfiguration('driver'),
            name=LaunchConfiguration('driver'),
            output=LaunchConfiguration('output')
        ),
    ])
