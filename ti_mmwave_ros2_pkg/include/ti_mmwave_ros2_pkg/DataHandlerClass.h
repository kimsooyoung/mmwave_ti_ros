#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_

#include "ti_mmwave_ros2_pkg/visibility_control.h"
#include "ti_mmwave_ros2_interfaces/msg/radar_scan.hpp"
// #include "more_interfaces/msg/address_book.hpp"
#include "ti_mmwave_ros2_pkg/mmWave.hpp"
// #include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <cstdlib>
#include <iostream>
// #include "ros/ros.h"
// #include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <visualization_msgs/msg/marker.hpp>
#define COUNT_SYNC_MAX 2

class DataUARTHandler : public rclcpp::Node {

public:
  /*Constructor*/
  // void DataUARTHandler(ros::NodeHandle* nh) :
  // currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
  COMPOSITION_PUBLIC
  // DataUARTHandler(ros::NodeHandle *nh);
  DataUARTHandler();

  COMPOSITION_LOCAL
  void setFrameID(char *myFrameID);

  /*User callable function to set the UARTPort*/
  COMPOSITION_LOCAL
  void setUARTPort(char *mySerialPort);

  /*User callable function to set the BaudRate*/
  COMPOSITION_LOCAL
  void setBaudRate(int myBaudRate);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  COMPOSITION_LOCAL
  void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  COMPOSITION_LOCAL
  void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

  COMPOSITION_LOCAL
  // void setNodeHandle(ros::NodeHandle *nh);

  /*User callable function to start the handler's internal threads*/
  COMPOSITION_LOCAL
  void start(void);

  /*Helper functions to allow pthread compatability*/
  COMPOSITION_LOCAL
  static void *readIncomingData_helper(void *context);

  COMPOSITION_LOCAL
  static void *sortIncomingData_helper(void *context);

  COMPOSITION_LOCAL
  static void *syncedBufferSwap_helper(void *context);

  /*Sorted mmwDemo Data structure*/
  mmwDataPacket mmwData;

private:
  int nr;
  int nd;
  int ntx;
  float fs;
  float fc;
  float BW;
  float PRI;
  float tfr;
  float max_range;
  float vrange;
  float max_vel;
  float vvel;

  char *frameID;
  /*Contains the name of the serial port*/
  char *dataSerialPort;

  /*Contains the baud Rate*/
  int dataBaudRate;

  /*Contains the max_allowed_elevation_angle_deg (points with elevation angles
    outside +/- max_allowed_elevation_angle_deg will be removed)*/
  int maxAllowedElevationAngleDeg;

  /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles
    outside +/- max_allowed_azimuth_angle_deg will be removed)*/
  int maxAllowedAzimuthAngleDeg;

  /*Mutex protected variable which synchronizes threads*/
  int countSync;

  /*Read/Write Buffers*/
  std::vector<uint8_t> pingPongBuffers[2];

  /*Pointer to current data (sort)*/
  std::vector<uint8_t> *currentBufp;

  /*Pointer to new data (read)*/
  std::vector<uint8_t> *nextBufp;

  /*Mutex protecting the countSync variable */
  pthread_mutex_t countSync_mutex;

  /*Mutex protecting the nextBufp pointer*/
  pthread_mutex_t nextBufp_mutex;

  /*Mutex protecting the currentBufp pointer*/
  pthread_mutex_t currentBufp_mutex;

  /*Condition variable which blocks the Swap Thread until signaled*/
  pthread_cond_t countSync_max_cv;

  /*Condition variable which blocks the Read Thread until signaled*/
  pthread_cond_t read_go_cv;

  /*Condition variable which blocks the Sort Thread until signaled*/
  pthread_cond_t sort_go_cv;

  /*Swap Buffer Pointers Thread*/
  COMPOSITION_LOCAL
  void *syncedBufferSwap(void);

  /*Checks if the magic word was found*/
  COMPOSITION_LOCAL
  int isMagicWord(uint8_t last8Bytes[8]);

  /*Read incoming UART Data Thread*/
  COMPOSITION_LOCAL
  void *readIncomingData(void);

  /*Sort incoming UART Data Thread*/
  COMPOSITION_LOCAL
  void *sortIncomingData(void);

  COMPOSITION_LOCAL
  void visualize(const ti_mmwave_ros2_interfaces::msg::RadarScan &msg);

  // ros::NodeHandle *nodeHandle;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr DataUARTHandler_pub;
  rclcpp::Publisher<ti_mmwave_ros2_interfaces::msg::RadarScan>::SharedPtr radar_scan_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

  // ros::Publisher DataUARTHandler_pub;
  // ros::Publisher radar_scan_pub;
  // ros::Publisher marker_pub;
};

#endif
