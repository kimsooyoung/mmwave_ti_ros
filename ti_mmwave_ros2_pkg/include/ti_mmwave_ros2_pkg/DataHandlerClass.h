#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_

#include "ti_mmwave_ros2_interfaces/msg/radar_scan.hpp"
#include "ti_mmwave_ros2_pkg/visibility_control.h"
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

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using RadarScan = ti_mmwave_ros2_interfaces::msg::RadarScan;
using Marker = visualization_msgs::msg::Marker;

class DataUARTHandler : public rclcpp::Node {

public:
  /*Constructor*/
  // void DataUARTHandler(ros::NodeHandle* nh) :
  // currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
  // DataUARTHandler(ros::NodeHandle *nh);
  COMPOSITION_PUBLIC
  DataUARTHandler();

  COMPOSITION_PUBLIC
  void getPublishers(
      const rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub_in,
      const rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub_in,
      const rclcpp::Publisher<Marker>::SharedPtr marker_pub_in);

  void setFrameID(char *myFrameID);

  /*User callable function to set the UARTPort*/
  void setUARTPort(char *mySerialPort);

  /*User callable function to set the BaudRate*/
  void setBaudRate(int myBaudRate);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

  /*User callable function to start the handler's internal threads*/
  void start(void);

  /*Helper functions to allow pthread compatability*/
  static void *readIncomingData_helper(void *context);

  static void *sortIncomingData_helper(void *context);

  static void *syncedBufferSwap_helper(void *context);

  void callbackGlobalParam(
      std::shared_future<std::vector<rclcpp::Parameter>> future);

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
  void *syncedBufferSwap(void);

  /*Checks if the magic word was found*/
  int isMagicWord(uint8_t last8Bytes[8]);

  /*Read incoming UART Data Thread*/
  void *readIncomingData(void);

  /*Sort incoming UART Data Thread*/
  void *sortIncomingData(void);

  void visualize(const RadarScan &msg);

  rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub;
  rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub;

  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
};

#endif
