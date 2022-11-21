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
    : Node("parameter_parser", options) {}

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

  if (!this->has_parameter("/ti_mmwave/doppler_vel_resolution")) {
    this->declare_parameter("/ti_mmwave/num_TX");
    this->declare_parameter("/ti_mmwave/f_s");
    this->declare_parameter("/ti_mmwave/f_c");
    this->declare_parameter("/ti_mmwave/BW");
    this->declare_parameter("/ti_mmwave/PRI");
    this->declare_parameter("/ti_mmwave/t_fr");
    this->declare_parameter("/ti_mmwave/max_range");
    this->declare_parameter("/ti_mmwave/range_resolution");
    this->declare_parameter("/ti_mmwave/max_doppler_vel");
    this->declare_parameter("/ti_mmwave/doppler_vel_resolution");
  }

  this->set_parameter(rclcpp::Parameter("/ti_mmwave/num_TX", ntx));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/f_s", fs));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/f_c", fc));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/BW", BW));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/PRI", PRI));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/t_fr", tfr));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/max_range", max_range));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/range_resolution", vrange));
  this->set_parameter(rclcpp::Parameter("/ti_mmwave/max_doppler_vel", max_vel));
  this->set_parameter(
      rclcpp::Parameter("/ti_mmwave/doppler_vel_resolution", vvel));

  std::cout << "/ti_mmwave/num_TX : " << ntx << std::endl;
  std::cout << "/ti_mmwave/f_s : " << fs << std::endl;
  std::cout << "/ti_mmwave/f_c : " << fc << std::endl;
  std::cout << "/ti_mmwave/BW : " << BW << std::endl;
  std::cout << "/ti_mmwave/PRI : " << PRI << std::endl;
  std::cout << "/ti_mmwave/t_fr : " << tfr << std::endl;
  std::cout << "/ti_mmwave/max_range : " << max_range << std::endl;
  std::cout << "/ti_mmwave/range_resolution : " << vrange << std::endl;
  std::cout << "/ti_mmwave/max_doppler_vel : " << max_vel << std::endl;
  std::cout << "/ti_mmwave/doppler_vel_resolution : " << vvel << std::endl;

}

void ParameterParser::printParam() {

}

} // namespace ti_mmwave_ros2_pkg

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::ParameterParser)