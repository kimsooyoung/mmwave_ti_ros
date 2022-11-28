import os

import launch
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir_path = get_package_share_directory('ti_mmwave_ros2_pkg')
    rviz_config_file = os.path.join(pkg_dir_path, 'rviz', 'mmwave_3d_view_multi.rviz')

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
    )

    return launch.LaunchDescription([
        rviz2
    ])