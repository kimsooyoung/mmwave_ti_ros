#include "rclcpp/rclcpp.hpp"

class TestParams : public rclcpp::Node {
public:
  TestParams() : Node("test_params_rclcpp") {
    this->declare_parameter("my_str", "test");
    this->declare_parameter("my_int", 18);
    this->declare_parameter("my_double", 20.0);

    // rclcpp::Parameter str_param = this->get_parameter("my_str");
    // rclcpp::Parameter int_param = this->get_parameter("my_int");
    // rclcpp::Parameter double_array_param =
    //     this->get_parameter("my_double_array");

    // rclcpp::Parameter str_param("my_str", "Hola from code");
    // this->set_parameter(str_param);
    // // you can also do:
    // this->set_parameter(rclcpp::Parameter("my_int", 77));
    // this->set_parameter(rclcpp::Parameter("my_double_array", std::vector<double>{8.8, 9.9}));
  }

private:
};
// Code below is just to start the node
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParams>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}