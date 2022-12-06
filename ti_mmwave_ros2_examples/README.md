
```
# go1 launch
ros2 launch unitree_legged_real eloquent_high_level.launch.py
ros2 run joy joy_node

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
# 통합됨
ros2 launch ti_mmwave_ros2_examples pointcloud_to_laserscan.launch.py

docker exec -it ros2_humble_arm64v8 bash
ros2 launch ti_mmwave_ros2_examples bringup_launch.py open_rviz:=false
```

```
ros2 run ti_mmwave_ros2_examples dummy_pointcloud_publisher
ros2 run ti_mmwave_ros2_examples pointcloud_to_laserscan_node
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 test scan_frame
ros2 topic echo /scan
```
=> subscriber가 있어야만 publish를 한다.

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=fuck scan:=ssss


pointcloud_to_laserscan_node 왜안될까?
=> header.stamp는 없어도 된다.
=> but, PointCloud2 단에서 fraimID는 target_frame과 맞아야 한다.

stamp 없는 문제
=> pointcloud_to_laserscan_node단의 now를 박아넣도록 하였다.

ROS2와 ROS1 header양식이 다르다.
http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html
https://docs.ros2.org/latest/api/std_msgs/msg/Header.html


```
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: /ti_mmwave_0
height: 1
width: 2
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: intensity
  offset: 16
  datatype: 7
  count: 1
is_bigendian: false
point_step: 32
row_step: 64
data: [127, 157, 128, 61, 79, 42, 35, 62, 20, 219, 34, 61, 0, 0, 128, 63, 205, 204, 76, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 77, 121, 148, 61, 79, 42, 35, 62, 40, 86, 134, 60, 0, 0, 128, 63, 154, 153, 57, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
is_dense: true
---
```

---

이런식으로 composition + node 생성이 가능하군
https://github.com/ros-perception/pointcloud_to_laserscan/blob/eloquent-devel/CMakeLists.txt

```
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: test
height: 1
width: 100
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
is_bigendian: false
point_step: 12
row_step: 1200
data: [224, 236, 249, 62, 160, 174, 109, 63, 160, 184, 9, 64, 128, 84, 92, 64, 128, 137, 131, 63, 208, 21, 101, 64, 80, 205, 229, 62, 184, 61, 94, 64, 152, 113, 67, 191, 92, 41, 158, 63, 148, 190, 186, 63, 204, 253, 147, 191, 216, 198, 31, 191, 238, 147, 1, 192, 24, 188, 122, 64, 19, 218, 141, 192, 64, 95, 148, 64, 254, 127, 17, 192, 228, 49, 149, 191, 128, 181, 100, 190, 60, 180, 58, 64, 184, 201, 71, 64, 32, 241, 147, 62, 160, 8, 77, 190, 176, 49, 46, 63, 70, 60, 137, 191, 224, 48, 136, 64, 36, 23, 87, 64, 186, 68, 137, 192, 6, 34, 208, 191, 94, 30, 132, 192, 248, 168, 189, 63, '...']
is_dense: false
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: test
height: 1
width: 100
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
is_bigendian: false
point_step: 12
row_step: 1200
data: [224, 236, 249, 62, 160, 174, 109, 63, 160, 184, 9, 64, 128, 84, 92, 64, 128, 137, 131, 63, 208, 21, 101, 64, 80, 205, 229, 62, 184, 61, 94, 64, 152, 113, 67, 191, 92, 41, 158, 63, 148, 190, 186, 63, 204, 253, 147, 191, 216, 198, 31, 191, 238, 147, 1, 192, 24, 188, 122, 64, 19, 218, 141, 192, 64, 95, 148, 64, 254, 127, 17, 192, 228, 49, 149, 191, 128, 181, 100, 190, 60, 180, 58, 64, 184, 201, 71, 64, 32, 241, 147, 62, 160, 8, 77, 190, 176, 49, 46, 63, 70, 60, 137, 191, 224, 48, 136, 64, 36, 23, 87, 64, 186, 68, 137, 192, 6, 34, 208, 191, 94, 30, 132, 192, 248, 168, 189, 63, '...']
is_dense: false
```
