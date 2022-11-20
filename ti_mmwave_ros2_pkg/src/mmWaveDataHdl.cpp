/*
* @file mmWaveDataHdl.cpp
*
* @brief
* Creates the data handler node and sets parameters.
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
#include "ti_mmwave_ros2_pkg/mmWaveDataHdl.hpp"
#include "ti_mmwave_ros2_pkg/DataHandlerClass.h"

namespace ti_mmwave_ros2_pkg
{

mmWaveDataHdl::mmWaveDataHdl(const rclcpp::NodeOptions & options) : Node("mmWaveDataHdl", options) {
    onInit();
}

void mmWaveDataHdl::onInit()
{
    std::string mySerialPort;
    std::string myFrameID;
    int myBaudRate;
    int myMaxAllowedElevationAngleDeg;
    int myMaxAllowedAzimuthAngleDeg;
   
    mySerialPort = this->declare_parameter("data_port", "/dev/ttyUSB0");
    myBaudRate = this->declare_parameter("data_rate", 115200);
    myFrameID = this->declare_parameter("frame_id", "/ti_mmwave_0");
    myMaxAllowedElevationAngleDeg = this->declare_parameter("max_allowed_elevation_angle_deg", 90);
    myMaxAllowedAzimuthAngleDeg = this->declare_parameter("max_allowed_azimuth_angle_deg", 90);

    RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_port = %s", mySerialPort.c_str());
    RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_rate = %d", myBaudRate);
    RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
    RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);
   
    // DataUARTHandler DataHandler;
    auto DataHandler = std::make_shared<DataUARTHandler>();
    DataHandler->setFrameID( (char*) myFrameID.c_str() );
    DataHandler->setUARTPort( (char*) mySerialPort.c_str() );
    DataHandler->setBaudRate( myBaudRate );
    DataHandler->setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
    DataHandler->setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
    
    rclcpp::spin(DataHandler);
   
    RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: Finished onInit function");
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::mmWaveDataHdl)