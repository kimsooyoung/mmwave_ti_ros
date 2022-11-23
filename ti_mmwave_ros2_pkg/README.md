미심쩍은 부분

```
void DataUARTHandler::start(void)
```

여기서 spin을 걸어뒀는데 나는 node안에 node를 만들어서 실행하도록 해두었다.

```
    auto DataHandler = std::make_shared<DataUARTHandler>();
    DataHandler->setFrameID( (char*) myFrameID.c_str() );
    DataHandler->setUARTPort( (char*) mySerialPort.c_str() );
    DataHandler->setBaudRate( myBaudRate );
    DataHandler->setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
    DataHandler->setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
    
    rclcpp::spin(DataHandler);
```

```
$ ros2 component types
...
ti_mmwave_ros2_pkg
  ti_mmwave_ros2_pkg::mmWaveDataHdl

# terminal 1
ros2 run rclcpp_components component_container
ros2 component list
/ComponentManager

# terminal 2
ros2 component load /ComponentManager ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveDataHdl

ros2 component load /ComponentManager ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveCommSrv

ros2 component unload /ComponentManager 1
```

```
ros2 launch ti_mmwave_ros2_pkg 
```

* Quick Config만

```
ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
=> 알아서 잘 꺼짐

ros2 run rclcpp_components component_container
ros2 component load /ComponentManager ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveCommSrv

ros2 component load /my_container ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveCommSrv


```

* mmWaveDataHdl
```
ros2 run rclcpp_components component_container

ros2 launch ti_mmwave_ros2_pkg eloquent_only_config.launch.py

ros2 component load /ComponentManager ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveDataHdl

ros2 component load /ComponentManager ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg::mmWaveCommSrv

ros2 component unload /ComponentManager 1

```

결과

```
==============================
DataUARTHandler Read Thread joined
DataUARTHandler Sort Thread joined
DataUARTHandler Swap Thread joined
[INFO] [mmWaveDataHdl]: mmWaveDataHdl: Finished onInit function
DataUARTHandler Read Thread: Port is open[INFO] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<ti_mmwave_ros2_pkg::ParameterParser>
[INFO] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<ti_mmwave_ros2_pkg::mmWaveCommSrv>
[INFO] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<ti_mmwave_ros2_pkg::mmWaveCommSrv>
[INFO] [mmWaveCommSrv]: mmWaveCommSrv: command_port = /dev/ttyUSB0
[INFO] [mmWaveCommSrv]: mmWaveCommSrv: command_rate = 115200
[INFO] [mmWaveCommSrv]: mmWaveCommsrv: Finished onInit function
component_container: ../nptl/pthread_mutex_lock.c:81: __pthread_mutex_lock: Assertion `mutex->__data.__owner == 0' failed.

```

```
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

ros2 run ti_mmwave_ros2_pkg ti_mmwave_ros2_pkg
=> 이거 하면 
DataUARTHandler Read Thread: Port is opensyncedBufferSwap
이거까지는 나온다.

ros2 run ti_mmwave_ros2_pkg mmwave_comm_srv_node
ros2 launch ti_mmwave_ros2_pkg eloquent_only_config.launch.py

```

* launch 파일 만들기
```
ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
```
* parameter 바꿀 수 있게 변경
```

```

* rviz
```
ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
use sim time true로 하면 점이 안사라진다.
```


```
ros2 launch ti_mmwave_ros2_pkg eloquent_composition.launch.py
```