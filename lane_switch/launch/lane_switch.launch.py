from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lane_switch",
                executable="lane_switch_node",
                name="lane_switch_node",
                parameters=[
                    {
                        "trigger_distance": 20.0,
                        "avoidance_duration": 1.5,
                        "transition_duration": 4.0,
                        "return_transition_duration": 3.0,
                        "left_lane_position_ratio": 0.3,
                        "path_extension_length": 50.0,
                        "path_extension_resolution": 1.0,
                    }
                ],
                # remappings=[
                #     ("/path", "/ZOE3/path_follower/setPath"),
                #     ("/prius/ground_truth", "/ZOE3/position/map_ekf_odometry"),
                # ],
                output="screen",
            )
        ]
    )
