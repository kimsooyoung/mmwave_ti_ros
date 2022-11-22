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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ti_mmwave_ros2_pkg/DataHandlerClass.h"

namespace ti_mmwave_ros2_pkg {

mmWaveDataHdl::mmWaveDataHdl(const rclcpp::NodeOptions &options)
    : Node("mmWaveDataHdl", options) {
  onInit();
}

void mmWaveDataHdl::onInit() {
  std::string mySerialPort;
  std::string myFrameID;
  int myBaudRate;
  int myMaxAllowedElevationAngleDeg;
  int myMaxAllowedAzimuthAngleDeg;

  mySerialPort = this->declare_parameter("data_port", "/dev/ttyUSB1");
  myBaudRate = this->declare_parameter("data_rate", 921600);
  myFrameID = this->declare_parameter("frame_id", "/ti_mmwave_0");
  myMaxAllowedElevationAngleDeg =
      this->declare_parameter("max_allowed_elevation_angle_deg", 90);
  myMaxAllowedAzimuthAngleDeg =
      this->declare_parameter("max_allowed_azimuth_angle_deg", 90);

  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_port = %s",
              mySerialPort.c_str());
  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_rate = %d", myBaudRate);
  RCLCPP_INFO(this->get_logger(),
              "mmWaveDataHdl: max_allowed_elevation_angle_deg = %d",
              myMaxAllowedElevationAngleDeg);
  RCLCPP_INFO(this->get_logger(),
              "mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d",
              myMaxAllowedAzimuthAngleDeg);

  // Wait for parameters
  // 여기서 교착 발생
  //   while (!this->has_parameter("/ti_mmwave/doppler_vel_resolution")) {
  //     std::cout << "waiting param" << std::endl;
  //     rclcpp::sleep_for(std::chrono::seconds(3));
  //   }

  // this->set_on_parameters_set_callback(std::bind(
  //     &mmWaveDataHdl::parametersCallback, this, std::placeholders::_1));

//   timer_ =
//       this->create_wall_timer(std::chrono::milliseconds(200),
//                               std::bind(&mmWaveDataHdl::timer_callback, this));

  callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &mmWaveDataHdl::parametersCallback, this, std::placeholders::_1));

  if (this->has_parameter("/ti_mmwave/doppler_vel_resolution")) {
    nr = this->get_parameter("/ti_mmwave/numAdcSamples").as_int();
    nd = this->get_parameter("/ti_mmwave/numLoops").as_int();
    ntx = this->get_parameter("/ti_mmwave/num_TX").as_int();

    fs = static_cast<float>(this->get_parameter("/ti_mmwave/f_s").as_double());
    fc = static_cast<float>(this->get_parameter("/ti_mmwave/f_c").as_double());

    BW = static_cast<float>(this->get_parameter("/ti_mmwave/BW").as_double());
    PRI = static_cast<float>(this->get_parameter("/ti_mmwave/PRI").as_double());
    tfr =
        static_cast<float>(this->get_parameter("/ti_mmwave/t_fr").as_double());

    max_range = static_cast<float>(
        this->get_parameter("/ti_mmwave/max_range").as_double());
    vrange = static_cast<float>(
        this->get_parameter("/ti_mmwave/range_resolution").as_double());
    max_vel = static_cast<float>(
        this->get_parameter("/ti_mmwave/max_doppler_vel").as_double());
    vvel = static_cast<float>(
        this->get_parameter("/ti_mmwave/doppler_vel_resolution").as_double());
  } else {
    nr = this->declare_parameter("/ti_mmwave/numAdcSamples", 240);
    nd = this->declare_parameter("/ti_mmwave/numLoops", 16);
    ntx = this->declare_parameter("/ti_mmwave/num_TX", 3);

    fs = this->declare_parameter("/ti_mmwave/f_s", 7.5e+06);
    fc = this->declare_parameter("/ti_mmwave/f_c", 6.23e+10);

    BW = this->declare_parameter("/ti_mmwave/BW", 3.2e+09);
    PRI = this->declare_parameter("/ti_mmwave/PRI", 8.1e-05);
    tfr = this->declare_parameter("/ti_mmwave/t_fr", 0.033333);

    max_range = this->declare_parameter("/ti_mmwave/max_range", 11.2422);
    vrange = this->declare_parameter("/ti_mmwave/range_resolution", 0.0468426);
    max_vel = this->declare_parameter("/ti_mmwave/max_doppler_vel", 9.90139);
    vvel =
        this->declare_parameter("/ti_mmwave/doppler_vel_resolution", 0.618837);
  }

  auto DataUARTHandler_pub =
      create_publisher<PointCloud2>("/ti_mmwave/radar_scan_pcl", 100);
  auto radar_scan_pub =
      create_publisher<RadarScan>("/ti_mmwave/radar_scan", 100);
  auto marker_pub =
      create_publisher<Marker>("/ti_mmwave/radar_scan_markers", 100);

  // 여기서 교착이 발생한다.
  //   DataUARTHandler DataHandler = DataUARTHandler();

  //   DataHandler.getPublishers(DataUARTHandler_pub, radar_scan_pub,
  //   marker_pub);

  //   DataHandler.setParameter(nr, nd, ntx, fs, fc, BW, PRI, tfr, max_range,
  //   vrange,
  //                            max_vel, vvel);

  //   DataHandler.setFrameID((char *)myFrameID.c_str());
  //   DataHandler.setUARTPort((char *)mySerialPort.c_str());
  //   DataHandler.setBaudRate(myBaudRate);
  //   DataHandler.setMaxAllowedElevationAngleDeg(myMaxAllowedElevationAngleDeg);
  //   DataHandler.setMaxAllowedAzimuthAngleDeg(myMaxAllowedAzimuthAngleDeg);
  //   DataHandler.start();

  DataHandler = std::make_shared<DataUARTHandler>();
  DataHandler->getPublishers(DataUARTHandler_pub, radar_scan_pub, marker_pub);

  DataHandler->setParameter(nr, nd, ntx, fs, fc, BW, PRI, tfr, max_range,
                            vrange, max_vel, vvel);

  DataHandler->setFrameID((char *)myFrameID.c_str());
  DataHandler->setUARTPort((char *)mySerialPort.c_str());
  DataHandler->setBaudRate(myBaudRate);
  DataHandler->setMaxAllowedElevationAngleDeg(myMaxAllowedElevationAngleDeg);
  DataHandler->setMaxAllowedAzimuthAngleDeg(myMaxAllowedAzimuthAngleDeg);

  // 이러면 빌드는 되지만 점유를 해버린다. => config등 아무것도 안됨
  // rclcpp::spin(DataHandler);
  DataHandler->start();

  // while (rclcpp::ok()) {
  //     rclcpp::spin_some(DataHandler);
  // }

  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: Finished onInit function");
}

rcl_interfaces::msg::SetParametersResult mmWaveDataHdl::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::cout << "SetParametersResult" << std::endl;

  // Here update class attributes, do some actions, etc.
  return result;
}

void mmWaveDataHdl::timer_callback() { DataHandler->start(); }

} // namespace ti_mmwave_ros2_pkg

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::mmWaveDataHdl)