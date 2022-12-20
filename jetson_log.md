
```
sudo apt-get update
sudo apt-get install libpcl-dev
```

## Known Error

* terminate called after throwing an instance of 'serial::SerialException'
```
change USB Hub
```

* Boost error

ref : https://stackoverflow.com/questions/18200300/undefined-reference-to-boostsystemgeneric-category

```
# check boost version
cat /usr/include/boost/version.hpp | grep "BOOST_LIB_VERSION"
# symbolic link reconfigure 
sudo ln -s /usr/lib/aarch64-linux-gnu/libboost_system.so.1.65.1 /usr/lib/libboost_system.so

sudo apt remove libboost-dev -y
sudo apt install libboost-dev -y
# rebuild ti_mmwave_ros2_pkg
```

* libnav2_compute_path_to_pose_action_bt_node.so: undefined symbol

ref : https://github.com/ros-planning/navigation2/issues/1948

```

```

sudo rm -i /etc/apt/sources.list.d/PPA_Name.list
# pcl common is required but boost was not found