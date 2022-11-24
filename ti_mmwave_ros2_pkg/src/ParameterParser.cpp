/*
 * @file ParameterParser.cpp
 *
 * @brief
 * Calculates parameters from QuickConfig.
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
#include "ti_mmwave_ros2_pkg/ParameterParser.h"

namespace ti_mmwave_ros2_pkg {

ParameterParser::ParameterParser(const rclcpp::NodeOptions &options)
    : Node("parameter_parser", options) {
  parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/mmWaveCommSrvNode");

  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "client interrupted while waiting for service to appear.");
      exit(0);
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
}

void ParameterParser::ParamsParser(const std::string &srv) {

  //   ROS_ERROR("%s",srv.request.comm.c_str());
  //   ROS_ERROR("%s",srv.response.resp.c_str());
  std::vector<std::string> v;
  std::string s = srv;
  std::istringstream ss(s);
  std::string token;
  std::string req;
  int i = 0;
  while (std::getline(ss, token, ' ')) {
    v.push_back(token);
    if (i > 0) {
      if (!req.compare("profileCfg")) {
        // profileCfg 0 77 39 7 57.14 0 0 70 1 240 4884 0 0 30
        switch (i) {
        case 2:
          startFreq = std::stof(token);
          break;
        case 3:
          idleTime = std::stof(token);
          break;
        case 4:
          adcStartTime = std::stof(token);
          break;
        case 5:
          rampEndTime = std::stof(token);
          break;
        case 8:
          freqSlopeConst = std::stof(token);
          break;
        case 10:
          numAdcSamples = std::stof(token);
          break;
        case 11:
          digOutSampleRate = std::stof(token);
          break;
        case 14:
          rxGain = std::stof(token);
          break;
        }
      } else if (!req.compare("frameCfg")) {
        switch (i) {
        case 1:
          chirpStartIdx = std::stoi(token);
          break;
        case 2:
          chirpEndIdx = std::stoi(token);
          break;
        case 3:
          numLoops = std::stoi(token);
          break;
        case 4:
          numFrames = std::stoi(token);
          break;
        case 5:
          framePeriodicity = std::stof(token);
          break;
        }
      }
    } else
      req = token;
    i++;
  }
}

void ParameterParser::CalParams() {
  float c0 = 299792458;

  int ntx = chirpEndIdx - chirpStartIdx + 1;
  int nd = numLoops;
  int nr = numAdcSamples;
  float tfr = framePeriodicity * 1e-3;
  float fs = digOutSampleRate * 1e3;
  float kf = freqSlopeConst * 1e12;
  float adc_duration = nr / fs;
  float BW = adc_duration * kf;
  float PRI = (idleTime + rampEndTime) * 1e-6;
  float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2);

  float vrange = c0 / (2 * BW);
  float max_range = nr * vrange;
  float max_vel = c0 / (2 * fc * PRI) / ntx;
  float vvel = max_vel / nd;

  rclcpp::Parameter nr_param("numAdcSamples", nr);
  rclcpp::Parameter nd_param("numLoops", nd);
  rclcpp::Parameter ntx_param("num_TX", ntx);
  rclcpp::Parameter fs_param("f_s", fs);
  rclcpp::Parameter fc_param("f_c", fc);
  rclcpp::Parameter BW_param("BW", BW);
  rclcpp::Parameter PRI_param("PRI", PRI);
  rclcpp::Parameter tfr_param("t_fr", tfr);
  rclcpp::Parameter max_range_param("max_range", max_range);
  rclcpp::Parameter vrange_param("range_resolution", vrange);
  rclcpp::Parameter max_vel_param("max_doppler_vel", max_vel);
  rclcpp::Parameter vvel_param("doppler_vel_resolution", vvel);

  auto parameters_future = parameters_client->set_parameters(
      {nr_param, nd_param, ntx_param, fs_param, fc_param, BW_param, PRI_param,
       tfr_param, max_range_param, vrange_param, max_vel_param, vvel_param},
      std::bind(&ParameterParser::callbackGlobalParam, this,
                std::placeholders::_1));

  std::cout << "numAdcSamples : " << nr << std::endl;
  std::cout << "numLoops : " << nd << std::endl;
  std::cout << "num_TX : " << ntx << std::endl;
  std::cout << "f_s : " << fs << std::endl;
  std::cout << "f_c : " << fc << std::endl;
  std::cout << "BW : " << BW << std::endl;
  std::cout << "PRI : " << PRI << std::endl;
  std::cout << "t_fr : " << tfr << std::endl;
  std::cout << "max_range : " << max_range << std::endl;
  std::cout << "range_resolution : " << vrange << std::endl;
  std::cout << "max_doppler_vel : " << max_vel << std::endl;
  std::cout << "doppler_vel_resolution : " << vvel << std::endl;
}

void ParameterParser::callbackGlobalParam(
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
        future) {
  RCLCPP_INFO(this->get_logger(), "Param Setup Done");
  rclcpp::shutdown();
}

} // namespace ti_mmwave_ros2_pkg

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::ParameterParser)