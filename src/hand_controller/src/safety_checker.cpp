#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

class SafetyChecker : public rclcpp::Node
{
public:
  SafetyChecker() : Node("safety_checker")
  {
    this->declare_parameter<std::string>("safety_yaml_path", "");

    // Subscribe to the LiDAR scan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&SafetyChecker::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Safety Checker Started. Monitoring /scan...");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::string safety_path;
    this->get_parameter("safety_yaml_path", safety_path);

    if (safety_path.empty()) return;

    // Check Front 60 Degrees
    
    bool blocked = false;
    float safe_dist = 0.60; // 60 cm stop distance

    auto check_index = [&](size_t i) {
        if (i < msg->ranges.size()) {
            float r = msg->ranges[i];
            if (r > 0.01 && r < safe_dist) {
                blocked = true;
            }
        }
    };

    // Check first 30 degrees
    for (size_t i = 0; i < 30; ++i) check_index(i);
    // Check last 30 degrees
    for (size_t i = msg->ranges.size() - 30; i < msg->ranges.size(); ++i) check_index(i);

    // --- WRITE TO FILE ---
    std::string status = blocked ? "BLOCKED" : "ALLOWED";

    // Only write if status CHANGED or every 1 second (to save disk IO)
    try {
        std::ofstream fout(safety_path);
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "status";
        out << YAML::Value << status;
        out << YAML::EndMap;
        fout << out.c_str();
    } catch (...) {
        // Ignore file errors for now
    }

    if (status != last_status_) {
        RCLCPP_INFO(this->get_logger(), "Safety Status: %s", status.c_str());
        last_status_ = status;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::string last_status_ = "UNKNOWN";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyChecker>());
  rclcpp::shutdown();
  return 0;
}