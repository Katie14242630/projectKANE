import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_pepper_autonomy = get_package_share_directory('pepper_autonomy')

    # Autonomy node
    autonomy_node_cmd = Node(
        package="pepper_autonomy",
        executable="autonomy_node",
        name="autonomy_node",
        parameters=[{
            "location_file": "/home/student/ros2_ws/src/pepperBotLibrary/src/pepper_autonomy/config/sim_lib_locations.yaml"
        }],
        output="screen"
    )

    # Object detector node
    object_detector_cmd = Node(
        package="pepper_autonomy",
        executable="object_detector",
        name="object_detector",
        output="screen"
    )

    # Create launch description and add both nodes
    ld = LaunchDescription()

    ld.add_action(autonomy_node_cmd)
    ld.add_action(object_detector_cmd)

    return ld
