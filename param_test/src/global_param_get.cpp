#include "rclcpp/rclcpp.hpp"

class TestGetGlobalParam : public rclcpp::Node {
public:
  TestGetGlobalParam() : Node("test_get_global_param") {

    parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "/test_params_rclcpp");

    parameters_client->wait_for_service();
    
    // reference
    // https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_async_parameters_client.html
    // get_parameters (
    //     const std::vector< std::string > &names, 
    //     std::function< void(std::shared_future< std::vector< rclcpp::Parameter >>) 
    // > callback=nullptr)
    auto parameters_future = parameters_client->get_parameters(
        {"my_str", "my_int", "my_double"}, std::bind(&TestGetGlobalParam::callbackGlobalParam,
                                       this, std::placeholders::_1));
  }
  void callbackGlobalParam(
      std::shared_future<std::vector<rclcpp::Parameter>> future) {
    auto result = future.get();
    auto string_param = result.at(0);
    auto int_param = result.at(1);
    auto double_param = result.at(2);

    RCLCPP_INFO(this->get_logger(), "Got global param: %s",
                string_param.as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Got global param: %d",
                int_param.as_int());
    RCLCPP_INFO(this->get_logger(), "Got global param: %f",
                double_param.as_double());
  }

private:
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestGetGlobalParam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}