import os
from time import sleep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  pkg_nav2_dir = get_package_share_directory('nav2_bringup')
  pkg_tb3_sim = get_package_share_directory('move_pepper')

  use_sim_time = LaunchConfiguration('use_sim_time', default='True')
  autostart = LaunchConfiguration('autostart', default='True')

  nav2_launch_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
      ),
      launch_arguments={
          'use_sim_time': use_sim_time,
          'autostart': autostart,
          'map': os.path.join(pkg_tb3_sim, 'maps', 'map.yaml')
      }.items()
  )

  rviz_launch_cmd = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=[
          '-d' + os.path.join(
              get_package_share_directory('nav2_bringup'),
              'rviz',
              'nav2_default_view.rviz'
          )]
  )

  set_init_amcl_pose_cmd = Node(
      package="move_pepper",
      executable="amcl_init_pose_publisher",
      name="amcl_init_pose_publisher",
      parameters=[{
          "x": 0.0,
          "y": 0.0,
      }]
  )

  # Static transform for external camera
  static_tf_camera_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_camera',
        arguments=['3', '0', '1', '0', '0', '0', 'map', 'camera_color_optical_frame']
    )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(nav2_launch_cmd)
  ld.add_action(set_init_amcl_pose_cmd)
  ld.add_action(rviz_launch_cmd)
  ld.add_action(static_tf_camera_cmd) #Add this to launch camera pose

  return ld