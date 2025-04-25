from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        robot_state_publisher_cmd
    ])
