from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to the existing explore launch file (from the other package)
    explore_launch = os.path.join(
        get_package_share_directory('explore_lite'),
        'launch',
        'explore.launch.py'
    )

    return LaunchDescription([
        # Launch explore_lite via its existing launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_launch)
        ),

        # Launch your object detection node
        Node(
            package='bme_gazebo_sensors_py',
            executable='object_detect',
            name='object_detection_node',
            output='screen'
        ),

        # Launch your controller
        Node(
            package='object_finder',
            executable='controller_node',
            name='controller_node',
            output='screen'
        )
    ])
