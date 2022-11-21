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

ros2 component unload /ComponentManager 1
```