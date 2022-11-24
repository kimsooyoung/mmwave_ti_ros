#include "rclcpp/rclcpp.hpp"

class TestSetGlobalParam : public rclcpp::Node {
public:
  TestSetGlobalParam() : Node("test_get_global_param") {

    parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "/test_params_rclcpp");

    parameters_client->wait_for_service();

    // reference
    // https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_async_parameters_client.html
    // set_parameters (
    //   const std::vector< rclcpp::Parameter > &parameters, 
    //   std::function< void(std::shared_future< std::vector< rcl_interfaces::msg::SetParametersResult >>) 
    // > callback=nullptr)

    rclcpp::Parameter str_param("my_str", "Hola from code");
    rclcpp::Parameter int_param("my_int", 777);

    auto parameters_future = parameters_client->set_parameters(
        {str_param, int_param}, std::bind(&TestSetGlobalParam::callbackGlobalParam,
                                       this, std::placeholders::_1));
  }
  void callbackGlobalParam(
      std::shared_future< std::vector< rcl_interfaces::msg::SetParametersResult >> future) {
    // auto result = future.get();
    // auto param = result.at(1);
    RCLCPP_INFO(this->get_logger(), "Done");
    
    // RCLCPP_INFO(this->get_logger(), "Got global param: %d",
    //             param.as_int());
    // RCLCPP_INFO(this->get_logger(), "Got global param: %s",
    //             param.as_string().c_str());
  }

private:
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestSetGlobalParam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}