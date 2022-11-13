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
#include "mmWaveDataHdl.hpp"
#include "DataHandlerClass.h"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

mmWaveDataHdl::mmWaveDataHdl() {}

void mmWaveDataHdl::onInit()
{
    ros::NodeHandle private_nh("~");

    std::string mySerialPort;
    std::string myFrameID;
    int myBaudRate;
    int myMaxAllowedElevationAngleDeg;
    int myMaxAllowedAzimuthAngleDeg;
   
    private_nh.getParam("data_port", mySerialPort);
   
    private_nh.getParam("data_rate", myBaudRate);
    
    private_nh.getParam("frame_id", myFrameID);

    if (!(private_nh.getParam("max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg))) {
        myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
    }

    if (!(private_nh.getParam("max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg))) {
        myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
    }

    ROS_INFO("mmWaveDataHdl: data_port = %s", mySerialPort.c_str());
    ROS_INFO("mmWaveDataHdl: data_rate = %d", myBaudRate);
    ROS_INFO("mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
    ROS_INFO("mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);
   
    DataUARTHandler DataHandler(&private_nh);
    DataHandler.setFrameID( (char*) myFrameID.c_str() );
    DataHandler.setUARTPort( (char*) mySerialPort.c_str() );
    DataHandler.setBaudRate( myBaudRate );
    DataHandler.setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
    DataHandler.setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
    DataHandler.start();
   
    NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
}

}