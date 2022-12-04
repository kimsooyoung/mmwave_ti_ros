#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("fake_laser_scan_node");
  auto pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());


  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  while(rclcpp::ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = 5;
      intensities[i] = 100 + count;
    }
    auto scan_time = node->get_clock()->now();

    //populate the LaserScan message
    sensor_msgs::msg::LaserScan dummy_scan;

    dummy_scan.header.stamp = scan_time;
    dummy_scan.header.frame_id = "fake_laser_frame";
    dummy_scan.angle_min = -1.57;
    dummy_scan.angle_max = 1.57;
    dummy_scan.angle_increment = 3.14 / num_readings;
    dummy_scan.time_increment = (1 / laser_frequency) / (num_readings);
    dummy_scan.range_min = 0.0;
    dummy_scan.range_max = 100.0;

    dummy_scan.scan_time = (1 / laser_frequency);

    dummy_scan.ranges.resize(num_readings);
    dummy_scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      dummy_scan.ranges[i] = ranges[i];
      dummy_scan.intensities[i] = intensities[i];
    }

    pub->publish(dummy_scan);
    ++count;
  }
}