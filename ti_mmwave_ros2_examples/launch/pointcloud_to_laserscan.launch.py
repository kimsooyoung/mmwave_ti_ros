import os

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import TimerAction, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cfg_file = "6843ISK_3d.cfg"
    namespace = ""

    mmwave_pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    this_pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_examples')
    cfg_file_path = os.path.join(mmwave_pkg_dir_path, 'cfg', cfg_file)
    rviz_config_file = os.path.join(this_pkg_dir_path, 'rviz', 'laser_scan.rviz')

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

    pointcloud_to_laserscan_node = Node(
        package='ti_mmwave_ros2_examples', 
        node_executable='pointcloud_to_laserscan_node',
        node_name='pointcloud_to_laserscan',
        remappings=[
            # ('cloud_in', ['/cloud']),
            ('cloud', ['/ti_mmwave/radar_scan_pcl']),
            # ('scan', ['/ssss']),
        ],
        parameters=[{
            'target_frame': 'scan_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -1.5707,  # -M_PI/2
            'angle_max': 1.5707,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.0,
            'range_max': 2.0,
            'use_inf': False,
            'inf_epsilon': 1.0
        }],
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'ti_mmwave_0', 'scan_frame']
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
        pointcloud_to_laserscan_node,
        static_transform_publisher,
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