colcon build --symlink-install --packages-select serial
source install/local_setup.bash

colcon build --symlink-install --packages-select ti_mmwave_ros2_interfaces
source install/local_setup.bash

colcon build --symlink-install --packages-select ti_mmwave_ros2_pkg
source install/local_setup.bash

# For this package, pcl common is required, But don't be afraid, ROS Installing contains PCL
colcon build --symlink-install --packages-select ti_mmwave_ros2_examples
source install/local_setup.bash