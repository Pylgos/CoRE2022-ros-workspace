from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    package_name = "bringup"
    share_dir = get_package_share_directory(package_name)
    
    all_without_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [share_dir + "/launch/all_without_visualizer.launch.py"]))
    
    return LaunchDescription([
        all_without_visualizer,
        Node(
            package="rviz2",
            namespace="",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "./default.rviz"]
        )
    ])
