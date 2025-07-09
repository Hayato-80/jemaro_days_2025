from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    downsampling_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("pointcloud_downsampling"),
                "launch",
                "pointcloud_downsampling.launch.py",
            )
        )
    )

    clean_node = Node(
        package="clean_point_cloud",
        executable="clean_point_cloud_node",
        name="clean_point_cloud_node",
        output="screen",
    )

    return LaunchDescription([downsampling_launch, clean_node])
