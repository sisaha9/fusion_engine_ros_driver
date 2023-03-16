import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description with multiple components."""
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )
    connection_type = DeclareLaunchArgument(
        "connection_type", default_value="serial", description="Takes values of serial or udp"
    )
    udp_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "udp.param.yaml")
    serial_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "udp.param.yaml")
    fusion_engine_ros_driver_param_file = os.path.join(get_package_share_directory('fusion_engine_ros_driver'), "param", "fusion_engine_ros_driver.param.yaml")

    fusion_engine_composable_node = ComposableNode(
        package='fusion_engine_ros_driver',
        plugin='point_one::fusion_engine::ros_driver::FusionEngineRosDriverNode',
        name='fusion_engine_ros_driver_node',
        parameters=[fusion_engine_ros_driver_param_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    udp_container = ComposableNodeContainer(
            name='udp_fusion_engine_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='udp_driver',
                    plugin='drivers::udp_driver::UdpReceiverNode',
                    name='udp_receiver_node',
                    remappings=[('/udp_read', '/fusion_engine_udp_packets')],
                    parameters=[udp_param_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
                fusion_engine_composable_node
            ],
            output='both',
            condition=LaunchConfigurationEquals("connection_type", "udp"),
    )
    serial_container = ComposableNodeContainer(
            name='serial_fusion_engine_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='serial_driver',
                    plugin='drivers::serial_driver::SerialBridgeNode',
                    name='serial_receiver_node',
                    remappings=[('/serial_read', '/fusion_engine_serial_packets')],
                    parameters=[serial_param_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
                fusion_engine_composable_node
            ],
            output='both',
            condition=LaunchConfigurationEquals("connection_type", "serial"),
    )

    return launch.LaunchDescription([use_sim_time, connection_type, udp_container, serial_container])