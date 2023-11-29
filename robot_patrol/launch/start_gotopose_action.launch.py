from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_patrol",
            executable="go_to_pose_action",
            output="screen",
            parameters = [{"use_sim_time" : True}],
            emulate_tty=True
        )
    ])