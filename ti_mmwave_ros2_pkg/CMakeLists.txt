cmake_minimum_required(VERSION 3.5)
project(ti_mmwave_ros2_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ti_mmwave_ros2_interfaces REQUIRED)

find_package(serial REQUIRED)

find_package(Boost REQUIRED)

find_package(Threads REQUIRED)

find_package(PCL 1.7.2 REQUIRED)

##### added ######

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${pthread_INCLUDE_DIRS}
  ${rt_INCLUDE_DIRS}
)

link_directories( ${Boost_LIBRARIES_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )

add_definitions(${PCL_DEFINITIONS})

add_library(mmwave SHARED 
  src/DataHandlerClass.cpp
  src/mmWaveDataHdl.cpp
)
target_compile_definitions(mmwave PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(mmwave
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  std_msgs
  # pcl_ros
  pcl_conversions
  ti_mmwave_ros2_interfaces
  serial
)
target_link_libraries(mmwave
  ${serial_LIBRARIES}
)
rclcpp_components_register_nodes(mmwave "ti_mmwave_ros2_pkg::mmWaveDataHdl")
set(node_plugins "${node_plugins}ti_mmwave_ros2_pkg::mmWaveDataHdl;$<TARGET_FILE:mmwave>\n")

install(TARGETS
  mmwave
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# # This CMake code is only required when you want to use
# # interfaces in the same package as the one in which they are used.
# rosidl_target_interfaces(mmwave
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
