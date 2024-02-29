import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import TimerAction, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cfg_file = "1642_2d.cfg"
    namespace = "radar_1"

    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    cfg_file_path = os.path.join(pkg_dir_path, 'cfg', cfg_file)
    rviz_config_file = os.path.join(pkg_dir_path, 'rviz', 'mmwave_3d_view.rviz')

    mmwave_quick_config = Node(
        package='ti_mmwave_ros2_pkg',
        node_executable='mmWaveQuickConfig',
        node_namespace=namespace,
        node_name='mmWaveQuickConfig',
        output='screen',
        arguments=[cfg_file_path],
        parameters=[{
            "mmWaveCLI_name": "mmWaveCLI",
            "namespace": namespace,
        }],
    )

    mmwave_comm_srv_node = Node(
        package='ti_mmwave_ros2_pkg',
        node_executable='mmwave_comm_srv_node',
        node_namespace=namespace,
        node_name='mmWaveCommSrvNode',
        output='screen',
        parameters=[{
            "command_port": "/dev/ttyUSB2",
            "command_rate": 115200,
            "mmWaveCLI_name": "mmWaveCLI",
        }],
    )
    
    # x y z yaw pitch roll frame_id child_frame_id
    static_transform_publisher = Node(
        package = "tf2_ros", 
        node_executable = "static_transform_publisher",
        arguments = "0 0 0 3.1415 0 0 ti_mmwave_pcl ti_mmwave_1".split()
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='mmwave_container2',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ti_mmwave_ros2_pkg',
                    node_plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                    node_name='mmWaveDataHdl',
                    parameters=[{
                        "data_port": "/dev/ttyUSB3",
                        "data_rate": 921600,
                        "frame_id": "ti_mmwave_1",
                        "max_allowed_elevation_angle_deg": 90,
                        "max_allowed_azimuth_angle_deg": 90,
                        "namespace": namespace,
                    }]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        mmwave_comm_srv_node,
        mmwave_quick_config,
        static_transform_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=mmwave_quick_config,
                on_exit=[container],
            )
        ),
    ])