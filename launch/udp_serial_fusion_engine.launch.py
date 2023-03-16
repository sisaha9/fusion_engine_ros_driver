import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description with multiple components."""
    ttl_dir_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )
    container = ComposableNodeContainer(
            name='image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_tools',
                    plugin='image_tools::Cam2Image',
                    name='cam2image',
                    remappings=[('/image', '/burgerimage')],
                    parameters=[{'width': 320, 'height': 240, 'burger_mode': True, 'history': 'keep_last'}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='image_tools',
                    plugin='image_tools::ShowImage',
                    name='showimage',
                    remappings=[('/image', '/burgerimage')],
                    parameters=[{'history': 'keep_last'}],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
    )

    return launch.LaunchDescription([container])