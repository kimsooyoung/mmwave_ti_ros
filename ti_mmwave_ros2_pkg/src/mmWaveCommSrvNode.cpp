/*
 * @file mmWaveCommSrv.cpp
 *
 * @brief
 * Communication service responsible for sending CLI commands to sensor.
 *
 * \par
 * NOTE:
 * (C) Copyright 2020 Texas Instruments, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_mmwave_ros2_pkg/visibility_control.h"
/*Include ROS specific headers*/
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"

// #include <pluginlib/class_list_macros.h>
// #include <nodelet/nodelet.h>

/*Include standard C/C++ headers*/
#include <cstdio>
#include <iostream>
#include <sstream>

/*mmWave Driver Headers*/
#include "ti_mmwave_ros2_interfaces/srv/mm_wave_cli.hpp"

class mmWaveCommSrv : public rclcpp::Node {
private:
  rclcpp::Service<ti_mmwave_ros2_interfaces::srv::MMWaveCLI>::SharedPtr commSrv;

  std::string mySerialPort;

  int myBaudRate;

  std::string mmWaveCLIName;

public:
  mmWaveCommSrv() : rclcpp::Node("mmWaveCommSrvNode") {
    // ros::NodeHandle private_nh = getPrivateNodeHandle();
    // ros::NodeHandle private_nh2("~"); // follow namespace for multiple
    // sensors

    mySerialPort = this->declare_parameter("command_port", "/dev/ttyUSB0");

    myBaudRate = this->declare_parameter("command_rate", 115200);
    mmWaveCLIName = this->declare_parameter("mmWaveCLI_name", "mmWaveCLI");

    RCLCPP_INFO(this->get_logger(), "mmWaveCommSrv: command_port = %s",
                mySerialPort.c_str());
    RCLCPP_INFO(this->get_logger(), "mmWaveCommSrv: command_rate = %d",
                myBaudRate);

    // service client
    commSrv = create_service<ti_mmwave_ros2_interfaces::srv::MMWaveCLI>(
        mmWaveCLIName, std::bind(&mmWaveCommSrv::commSrv_cb, this,
                                 std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "mmWaveCommsrv: Finished onInit function");
  }

  void commSrv_cb(
      std::shared_ptr<ti_mmwave_ros2_interfaces::srv::MMWaveCLI::Request> req,
      std::shared_ptr<ti_mmwave_ros2_interfaces::srv::MMWaveCLI::Response>
          res) {

    RCLCPP_DEBUG(this->get_logger(),
                 "mmWaveCommSrv: Port is \"%s\" and baud rate is %d",
                 mySerialPort.c_str(), myBaudRate);

    /*Open Serial port and error check*/
    serial::Serial mySerialObject("", myBaudRate,
                                  serial::Timeout::simpleTimeout(1000));
    mySerialObject.setPort(mySerialPort.c_str());
    try {
      mySerialObject.open();
    } catch (std::exception &e1) {
      RCLCPP_INFO(
          this->get_logger(),
          "mmWaveCommSrv: Failed to open User serial port with error: %s",
          e1.what());
      RCLCPP_INFO(this->get_logger(),
                  "mmWaveCommSrv: Waiting 20 seconds before trying again...");
      try {
        // Wait 20 seconds and try to open serial port again
        rclcpp::sleep_for(std::chrono::seconds(20));
        mySerialObject.open();
      } catch (std::exception &e2) {
        RCLCPP_ERROR(this->get_logger(),
                     "mmWaveCommSrv: Failed second time to open User serial "
                     "port, error: %s",
                     e1.what());
        RCLCPP_INFO(
            this->get_logger(),
            "mmWaveCommSrv: Port could not be opened. Port is \"%s\" and "
            "baud rate is %d",
            mySerialPort.c_str(), myBaudRate);
        //   return false;
      }
    }

    /*Read any previous pending response(s)*/
    while (mySerialObject.available() > 0) {
      mySerialObject.readline(res->resp, 1024, ":/>");
      RCLCPP_INFO(
          this->get_logger(),
          "mmWaveCommSrv: Received (previous) response from sensor: '%s'",
          res->resp.c_str());
      res->resp = "";
    }

    /*Send out command received from the client*/
    RCLCPP_INFO(this->get_logger(),
                "mmWaveCommSrv: Sending command to sensor: '%s'",
                req->comm.c_str());

    req->comm.append("\n");
    mySerialObject.write(req->comm.c_str());

    /*Read output from mmwDemo*/
    mySerialObject.readline(res->resp, 1024, ":/>");
    RCLCPP_INFO(this->get_logger(),
                "mmWaveCommSrv: Received response from sensor: '%s'",
                res->resp.c_str());

    mySerialObject.close();
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto mmwave_comm_srv_node = std::make_shared<mmWaveCommSrv>();

  rclcpp::spin(mmwave_comm_srv_node);
  rclcpp::shutdown();

  return 0;
}