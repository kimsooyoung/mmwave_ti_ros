#ifndef _PARAM_PARSER_CLASS_
#define _PARAM_PARSER_CLASS_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ti_mmwave_ros2_interfaces/srv/mm_wave_cli.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace ti_mmwave_ros2_pkg {

class ParameterParser : public rclcpp::Node {

public:
  explicit ParameterParser(const rclcpp::NodeOptions &options);

  void init(const std::string &ns);

  void ParamsParser(const std::string &srv);

  void CalParams();

  void callbackGlobalParam(
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
          future);

private:
  float startFreq;
  float idleTime;
  float adcStartTime;
  float rampEndTime;
  float freqSlopeConst;
  float numAdcSamples;
  float digOutSampleRate;
  float rxGain;

  int chirpStartIdx;
  int chirpEndIdx;
  int numLoops;
  int numFrames;
  float framePeriodicity;

  std::shared_ptr<rclcpp::AsyncParametersClient>
      parameters_client;
};

} // namespace ti_mmwave_ros2_pkg

#endif
