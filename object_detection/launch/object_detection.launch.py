from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the directory of the current launch file
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    clean_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("clean_point_cloud"),
                "launch",
                "clean_pointcloud_with_downsampling.launch.py",
            )
        )
    )

    cone_detection_node = Node(
        package="object_detection",
        executable="cone_detection_pcl",
        name="cone_detection_pcl",
        output="screen",
        # parameters=[
        #     os.path.join(
        #         get_package_share_directory("object_detection"),
        #         "config",
        #         "cone_detection_params.yaml",
        #     )
        # ],
    )

    # RViz2 node with config file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(launch_dir, "config_zoe.rviz")],
        output="screen",
    )

    return LaunchDescription([clean_launch, cone_detection_node, rviz_node])
