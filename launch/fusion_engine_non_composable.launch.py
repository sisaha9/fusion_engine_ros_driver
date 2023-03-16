# Copyright 2022 Siddharth Saha

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )
    connection_type = DeclareLaunchArgument(
        "connection_type", default_value="serial", description="Takes values of serial or udp"
    )
    udp_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "udp.param.yaml")
    serial_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "serial.param.yaml")
    fusion_engine_ros_driver_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "fusion_engine_ros_driver.param.yaml")
    return LaunchDescription(
        [
            use_sim_time,
            connection_type,
            Node(
                package="serial_driver",
                executable="serial_bridge",
                name="serial_receiver_node",
                output="screen",
                parameters=[serial_param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("/serial_read", "/fusion_engine_serial_packets"),
                ],
                emulate_tty=True,
                condition=LaunchConfigurationEquals("connection_type", "serial"),
            ),
            Node(
                package="udp_driver",
                executable="udp_receiver_node_exe",
                name="udp_receiver_node",
                output="screen",
                parameters=[udp_param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("/udp_read", "/fusion_engine_udp_packets"),
                ],
                emulate_tty=True,
                condition=LaunchConfigurationEquals("connection_type", "udp"),
            ),
            Node(
                package="fusion_engine_ros_driver",
                executable="fusion_engine_ros_driver_node_exe",
                name="fusion_engine_ros_driver_node",
                output="screen",
                parameters=[fusion_engine_ros_driver_param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                emulate_tty=True,
            ),
        ]
    )
