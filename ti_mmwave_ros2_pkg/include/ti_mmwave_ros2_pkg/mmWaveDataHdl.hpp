/*
 * mmWaveDataHdl.hpp
 *
 * This file defines a ROS nodelet which will open up a serial port provided by
 * the user at a certain baud rate (also provided by user) that will interface
 * with the 1443EVM mmwDemo Data UART to be used for board configuration.
 *
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef MMWAVE_DATA_HDL_H
#define MMWAVE_DATA_HDL_H

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
#include "ti_mmwave_ros2_pkg/DataHandlerClass.h"

namespace ti_mmwave_ros2_pkg {

class mmWaveDataHdl : public rclcpp::Node {
public:
  explicit mmWaveDataHdl(const rclcpp::NodeOptions &options);
  // rcl_interfaces::msg::SetParametersResult
  // parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

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

  void onInit();

  // char* mySerialPort;

  // int myBaudRate;

}; // Class mmWaveDataHdl

} // namespace ti_mmwave_ros2_pkg

#endif
