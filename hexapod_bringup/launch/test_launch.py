import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hexapod_teleop_joystick',
            executable='hexapod_teleop_joystick',
            name='teleop_joystick',
            parameters=[{'file': 'hexapod_description/params/joystick.yaml'}]
        )
    ])
