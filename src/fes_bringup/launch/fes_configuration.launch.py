from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    configuration_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "source ~/BIRD/ros2_ws_2/install/setup.bash && "
            "ros2 run fes_bringup configuration_node; exec bash",
        ],
        output="screen",
    )

    return LaunchDescription([
        configuration_node,
    ])