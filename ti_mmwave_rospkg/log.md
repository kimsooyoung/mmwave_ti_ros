quick config 빼고 ti_mmwave_rospkg만 돌리면

```
[ INFO] [1669091643.863230422]: Initializing nodelet with 8 worker threads.
[ INFO] [1669091643.867362559]: mmWaveCommSrv: command_port = /dev/ttyUSB0
[ INFO] [1669091643.867395048]: mmWaveCommSrv: command_rate = 115200
[ INFO] [1669091643.869427409]: mmWaveDataHdl: data_port = /dev/ttyUSB1
[ INFO] [1669091643.869454047]: mmWaveDataHdl: data_rate = 921600
[ INFO] [1669091643.869479973]: mmWaveDataHdl: max_allowed_elevation_angle_deg = 90
[ INFO] [1669091643.869503491]: mmWaveDataHdl: max_allowed_azimuth_angle_deg = 90
=> 여기서 멈춤
```

DataHandlerClass.cpp에서 `ros::spin();`을 지우면

```
==============================
List of parameters
==============================
Number of range samples: 240
Number of chirps: 16
f_s: 7.500 MHz
f_c: 62.300 GHz
Bandwidth: 3200.000 MHz
PRI: 81.000 us
Frame time: 33.333 ms
Max range: 11.242 m
Range resolution: 0.047 m
Max Doppler: +-4.951 m/s
Doppler resolution: 0.619 m/s
==============================

[ INFO] [1669091754.421785210]: DataUARTHandler Read Thread joined
[ INFO] [1669091754.421813947]: DataUARTHandler Sort Thread joined
[ INFO] [1669091754.421831493]: DataUARTHandler Swap Thread joined
[ INFO] [1669091754.422935150]: DataUARTHandler Read Thread: Port is open
[radar_0/ti_mmwave_config-3] process has finished cleanly
log file: /home/kimsooyoung/.ros/log/254c1128-6a1f-11ed-828e-9cb6d08bf543/radar_0-ti_mmwave_config-3*.log
```

