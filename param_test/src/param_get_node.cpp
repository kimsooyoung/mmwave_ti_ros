#include "rclcpp/rclcpp.hpp"

class TestParams : public rclcpp::Node {
public:
  TestParams() : Node("test_params_rclcpp") {

    std::string str_param = this->get_parameter("my_str").as_string();
    int int_param = static_cast<int>(this->get_parameter("my_int").as_int());
    double double_param = this->get_parameter("my_double").as_double();

    RCLCPP_INFO(this->get_logger(), "str_param: %s", str_param);
    RCLCPP_INFO(this->get_logger(), "int_param: %d", int_param);
    RCLCPP_INFO(this->get_logger(), "double_param: %f", double_param);

    // rclcpp::Parameter str_param = this->get_parameter("my_str");
    // rclcpp::Parameter int_param = this->get_parameter("my_int");
    // rclcpp::Parameter double_array_param =
    //     this->get_parameter("my_double");

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