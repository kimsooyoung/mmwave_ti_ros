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

    cfg_file = "6843ISK_3d.cfg"

    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    cfg_file_path = os.path.join(pkg_dir_path, 'cfg', cfg_file)
    rviz_config_file = os.path.join(pkg_dir_path, 'rviz', 'mmwave_3d_view.rviz')

    mmwave_quick_config = Node(
        package='ti_mmwave_ros2_pkg',
        node_executable='mmWaveQuickConfig',
        node_name='mmwave_quick_config',
        output='screen',
        arguments=[cfg_file_path],
        parameters=[{
            "mmWaveCLI_name": "/mmWaveCLI",
        }],
    )

    mmwave_comm_srv_node = Node(
        package='ti_mmwave_ros2_pkg',
        node_executable='mmwave_comm_srv_node',
        node_name='mmWaveCommSrvNode',
        output='screen',
        parameters=[{
            "command_port": "/dev/ttyUSB0",
            "command_rate": 115200,
            "mmWaveCLI_name": "/mmWaveCLI",
        }],
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='my_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ti_mmwave_ros2_pkg',
                    node_plugin='ti_mmwave_ros2_pkg::mmWaveDataHdl',
                    node_name='mmWaveDataHdl',
                    parameters=[{
                        "data_port": "/dev/ttyUSB1",
                        "data_rate": 921600,
                        "frame_id": "ti_mmwave_0",
                        "max_allowed_elevation_angle_deg": 90,
                        "max_allowed_azimuth_angle_deg": 90,
                    }]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        mmwave_comm_srv_node,
        mmwave_quick_config,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=mmwave_quick_config,
                on_exit=[container],
            )
        ),
        TimerAction(    
            period=3.0,
            actions=[rviz2]
        ),
    ])
