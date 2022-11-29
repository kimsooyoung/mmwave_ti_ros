
```
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
ros2 launch ti_mmwave_ros2_examples pointcloud_to_laserscan.launch.py
```

```
ros2 run ti_mmwave_ros2_examples dummy_pointcloud_publisher
ros2 run ti_mmwave_ros2_examples pointcloud_to_laserscan_node
ros2 topic echo /scan
```
=> subscriber가 있어야만 publish를 한다.

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=fuck scan:=ssss


pointcloud_to_laserscan_node 왜안될까?
=> header.stamp는 없어도 된다.
=> but, PointCloud2 단에서 fraimID는 target_frame과 맞아야 한다.


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

