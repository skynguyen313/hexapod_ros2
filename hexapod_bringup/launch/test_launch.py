# import launch
# import launch_ros.actions

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='hexapod_teleop_joystick',
#             executable='teleop_joystick',
#             parameters=['hexapod_description/params/joystick.yaml']
#         )
#     ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('hexapod_bringup'),
      'config',
      'joystick.yaml'
      )

   return LaunchDescription([
      Node(
         package='hexapod_teleop_joystick',
         executable='hexapod_teleop_joystick',
         parameters=[config],
         output='screen',
      )
   ])