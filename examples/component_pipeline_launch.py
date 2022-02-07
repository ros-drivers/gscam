"""
Example pipeline using rclcpp_components.

This launches the gscam and other nodes into a container so that they run in the same process.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gscam_config = 'videotestsrc is-live=true ! video/x-raw ! videoconvert'
    camera_info_url='package://gscam/examples/uncalibrated_parameters.ini'

    return LaunchDescription([ComposableNodeContainer(
        name='gscam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # GSCam driver
            ComposableNode(
                package='gscam',
                plugin='gscam::GSCam',
                name='gscam_node',
                parameters=[{
                    'gscam_config': gscam_config,
                    'camera_info_url': camera_info_url,
                }],
                # Future-proof: enable zero-copy IPC when it is available
                # https://github.com/ros-perception/image_common/issues/212
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # Bayer color decoding
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # Mono rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='mono_rectify_node',
                namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('image', 'image_mono'),
                    ('image_rect', 'image_rect_mono'),
                ],
            ),

            # Color rectification
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='color_rectify_node',
                namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('image', 'image_color'),
                    ('image_rect', 'image_rect_color'),
                ],
            ),
        ],
        output='screen',
    )])
