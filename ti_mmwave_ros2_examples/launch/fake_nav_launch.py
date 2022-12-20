import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_transform_publisher_scan = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'test']
    )

    static_transform_publisher_base_fp = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'base_link']
    )

    static_transform_publisher_odom = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint']
    )

    dummy_pointcloud_publisher = Node(
        package='ti_mmwave_ros2_examples', 
        node_executable='dummy_pointcloud_publisher',
        node_name='dummy_pointcloud_publisher',
        remappings=[
            # ('cloud_in', ['/cloud']),
            ('cloud', ['scan']),
        ],
        parameters=[{
            'cloud_frame_id': 'test',
        }],
    )

    return launch.LaunchDescription([
        static_transform_publisher_scan,
        static_transform_publisher_base_fp,
        static_transform_publisher_odom,
        dummy_pointcloud_publisher,
    ])