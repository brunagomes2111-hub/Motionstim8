from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def spawner(name, inactive=False):
    arguments = [name]

    if inactive:
        arguments.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
    )


def generate_launch_description():

    bringup_share = get_package_share_directory("fes_bringup")

    urdf_file = os.path.join(
        bringup_share,
        "urdf",
        "motionstim8.urdf",
    )

    controllers_file = os.path.join(
        bringup_share,
        "config",
        "controllers.yaml",
    )

    with open(urdf_file, "r") as file:
        robot_description = file.read()

    control_signals_node = Node(
        package="control_signals",
        executable="control_signals_node",
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node", 
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = spawner("joint_state_broadcaster")

    ankle_left_spawner = spawner("ankle_left_controller", inactive=True)
    knee_left_spawner = spawner("knee_left_controller", inactive=True)
    ankle_right_spawner = spawner("ankle_right_controller", inactive=True)
    knee_right_spawner = spawner("knee_right_controller", inactive=True)


    return LaunchDescription([
    robot_state_publisher,
    controller_manager,

    joint_state_broadcaster_spawner,

    ankle_left_spawner,
    knee_left_spawner,
    ankle_right_spawner,
    knee_right_spawner,

    control_signals_node

    ])
       
