import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='my_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='ti_mmwave_ros2_pkg',
                #     node_plugin='ti_mmwave_ros2_pkg::mmWaveCommSrv',
                #     node_name='mmWaveCommSrv',
                #     parameters=[{
                #         "command_port": "/dev/ttyUSB0",
                #         "command_rate": 115200,
                #         "mmWaveCLI_name": "/mmWaveCLI",
                #     }]
                # ),
                ComposableNode(
                    package='ti_mmwave_ros2_pkg',
                    node_plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                    node_name='mmWaveDataHdl',
                    parameters=[{
                        "data_port": "/dev/ttyUSB1",
                        "data_rate": 921600,
                        "frame_id": "/ti_mmwave_0",
                        "max_allowed_elevation_angle_deg": 90,
                        "max_allowed_azimuth_angle_deg": 90,
                    }]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])