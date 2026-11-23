import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Minimal example: two SCS servos (id 1/2) on one bus via ros2_control."""
    pkg_share = get_package_share_directory("feetech_scs_ros2_control")
    xacro_file = os.path.join(pkg_share, "urdf", "feetech_scs_example.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "feetech_controllers.yaml")

    usb_port = LaunchConfiguration("usb_port")
    baud_rate = LaunchConfiguration("baud_rate")
    thread_priority = LaunchConfiguration("thread_priority")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file, " ",
            "usb_port:=", usb_port, " ",
            "baud_rate:=", baud_rate,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "thread_priority": ParameterValue(thread_priority, value_type=int),
            },
            controllers_file,
        ],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "usb_port", default_value="/dev/ttyUSB0",
            description="SCS bus serial device (e.g. /dev/ttyUSB0, /dev/ttyS2)."),
        DeclareLaunchArgument(
            "baud_rate", default_value="1000000",
            description="SCS bus baud rate (must match every servo on the bus)."),
        DeclareLaunchArgument(
            "thread_priority", default_value="50",
            description="controller_manager RT thread priority (0 disables SCHED_FIFO)."),
        robot_state_publisher,
        controller_manager,
        TimerAction(period=4.0, actions=spawners),
    ])
