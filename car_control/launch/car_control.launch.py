from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="car_control",
                executable="pure_pursuit.py",
                name="pure_pursuit",
                output="screen",
                parameters=[
                    {
                        "max_steering": 1.22,
                        "look_ahead_dist": 2.5,
                        "kpp": 0.3,
                        "angle_offset": 1.5707963,
                        "plot_path": True,
                    }
                ],
                remappings=[("/odom", "/prius/ground_truth")],
            ),
        ]
    )
