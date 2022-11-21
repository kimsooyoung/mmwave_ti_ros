/*
 * @file mmWaveQuickConfig.cpp
 *
 * @brief
 * Reads the cfg file and calls service to send commands.
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

#include "rclcpp/rclcpp.hpp"
#include "ti_mmwave_ros2_interfaces/srv/mm_wave_cli.hpp"

#include <cstdlib>
#include <fstream>
#include <regex>
#include <stdio.h>

#include "ti_mmwave_ros2_pkg/ParameterParser.h"

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // std::cout << "argc : " << argc << std::endl;

  if (argc < 3) {
    std::cout << "mmWaveQuickConfig: usage: mmWaveQuickConfig "
                 "/file_directory/params.cfg"
              << std::endl;
    return 1;
  } else
    std::cout
        << "mmWaveQuickConfig: Configuring mmWave device using config file: "
        << argv[1] << std::endl;

  auto node = rclcpp::Node::make_shared("mmWaveQuickConfig");

  std::string mmWaveCLIName;
  mmWaveCLIName = node->declare_parameter("mmWaveCLI_name", "/mmWaveCLI");
  auto client = node->create_client<ti_mmwave_ros2_interfaces::srv::MMWaveCLI>(
      mmWaveCLIName);

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }

  auto request =
      std::make_shared<ti_mmwave_ros2_interfaces::srv::MMWaveCLI::Request>();

  // ti_mmwave_rospkg::mmWaveCLI srv;

  // if (argc != 2) {
  //     ROS_INFO("mmWaveQuickConfig: usage: mmWaveQuickConfig
  //     /file_directory/params.cfg"); return 1;
  // } else
  //     ROS_INFO("mmWaveQuickConfig: Configuring mmWave device using config
  //     file: %s", argv[1]);

  // std::string mmWaveCLIName;
  // private_nh.getParam("mmWaveCLI_name", mmWaveCLIName);

  // ros::ServiceClient client =
  // nh.serviceClient<ti_mmwave_rospkg::mmWaveCLI>(mmWaveCLIName);

  std::ifstream myParams;

  auto parser = std::make_shared<ti_mmwave_ros2_pkg::ParameterParser>(options);
  exec.add_node(parser);

  // //wait for service to become available
  // ros::service::waitForService(mmWaveCLIName, 10000);

  // wait 0.5 secs to avoid multi-sensor conflicts
  // ros::Duration(0.5).sleep();
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  myParams.open(argv[1]);

  if (myParams.is_open()) {
    while (std::getline(myParams, request->comm)) {
      // Remove Windows carriage-return if present
      request->comm.erase(
          std::remove(request->comm.begin(), request->comm.end(), '\r'),
          request->comm.end());
      // Ignore comment lines (first non-space char is '%') or blank lines
      if (!(std::regex_match(request->comm, std::regex("^\\s*%.*")) ||
            std::regex_match(request->comm, std::regex("^\\s*")))) {
        // ROS_INFO("mmWaveQuickConfig: Sending command: '%s'",
        // request->comm.c_str() );

        std::cout << "request->comm : " << request->comm << std::endl;
        auto result_future = client->async_send_request(request);

        // foxy : rclcpp::FutureReturnCode::SUCCESS
        // eloquent : rclcpp::executor::FutureReturnCode::SUCCESS
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "service call failed :(");
          // client->remove_pending_request(result_future);
          return 1;
        }
        auto result = result_future.get();

        if (result != nullptr) {
          if (std::regex_search(result->resp, std::regex("Done"))) {
            // ROS_INFO("mmWaveQuickConfig: Command successful (mmWave sensor
            // responded with 'Done')");
            parser->ParamsParser(result->resp);
          } else {
            RCLCPP_ERROR(node->get_logger(),
                         "mmWaveQuickConfig: Command failed (mmWave sensor did "
                         "not respond with 'Done')");
            RCLCPP_ERROR(node->get_logger(),
                         "mmWaveQuickConfig: Response: '%s'", result->resp);
            return 1;
          }
        } else {
          RCLCPP_ERROR(node->get_logger(),
                       "mmWaveQuickConfig: Failed to call service mmWaveCLI");
          RCLCPP_ERROR(node->get_logger(), "%s", request->comm.c_str());
          return 1;
        }
      }
    }
    parser->CalParams();
    myParams.close();
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "mmWaveQuickConfig: Failed to open File %s", argv[1]);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "mmWaveQuickConfig: mmWaveQuickConfig will now terminate. Done "
              "configuring mmWave device using config file: %s",
              argv[1]);

  rclcpp::shutdown();

  return 0;
}