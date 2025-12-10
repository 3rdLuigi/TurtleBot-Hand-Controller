#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem> // Requires C++17

class HandController : public rclcpp::Node
{
public:
  HandController() : Node("hand_controller")
  {
    // 1. Declare the parameter "yaml_path" with a default value (empty)
    this->declare_parameter<std::string>("yaml_path", "");

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HandController::control_loop, this));
      
    RCLCPP_INFO(this->get_logger(), "Hand Controller (C++) Started.");
  }

private:
  void control_loop()
  {
    std::string command = "STOP";
    
    // 2. Get the current value of the parameter
    std::string yaml_path;
    this->get_parameter("yaml_path", yaml_path);

    if (yaml_path.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "No YAML path provided! Use: ros2 run ... --ros-args -p yaml_path:=/path/to/file");
        return;
    }

    try {
      YAML::Node config = YAML::LoadFile(yaml_path);
      if (config["command"]) {
        command = config["command"].as<std::string>();
      }
    } catch (const YAML::BadFile& e) {
       // File might be busy writing
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Parse Error: %s", e.what());
    }

    auto msg = geometry_msgs::msg::Twist();

    // Logic Engine
    if (command == "MOVE_FORWARD") {
      msg.linear.x = 0.2;
    } else if (command == "MOVE_BACKWARD") {
      msg.linear.x = -0.2;
    } else if (command == "MOVE_LEFT" || command == "ROTATE_LEFT") {
      msg.angular.z = 0.5;
    } else if (command == "MOVE_RIGHT" || command == "ROTATE_RIGHT") {
      msg.angular.z = -0.5;
    } else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandController>());
  rclcpp::shutdown();
  return 0;
}