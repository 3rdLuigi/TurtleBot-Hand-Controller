#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <fstream>

class HandController : public rclcpp::Node
{
public:
  HandController() : Node("hand_controller")
  {
    this->declare_parameter<std::string>("motion_yaml_path", "");
    this->declare_parameter<std::string>("safety_yaml_path", "");

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Run at 10Hz (0.1s)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HandController::control_loop, this));
      
    RCLCPP_INFO(this->get_logger(), "Hand Controller Started. Waiting for commands...");
  }

private:
  void control_loop()
  {
    std::string motion_path, safety_path;
    this->get_parameter("motion_yaml_path", motion_path);
    this->get_parameter("safety_yaml_path", safety_path);

    if (motion_path.empty() || safety_path.empty()) return;

    std::string command = "STOP";
    std::string safety = "ALLOWED";

    // Read Motion Command
    try {
      YAML::Node config = YAML::LoadFile(motion_path);
      if (config["command"]) command = config["command"].as<std::string>();
    } catch (...) {}

    // Read Safety Status
    try {
      YAML::Node safe_config = YAML::LoadFile(safety_path);
      if (safe_config["status"]) safety = safe_config["status"].as<std::string>();
    } catch (...) {}

    // Logic Engine
    auto msg = geometry_msgs::msg::Twist();
    
    // Safety Override: If BLOCKED, you CANNOT go FORWARD.
    if (command == "MOVE_FORWARD") {
        if (safety == "BLOCKED") {
            RCLCPP_WARN(this->get_logger(), "Obstacle Detected! Stopping Forward Motion.");
            msg.linear.x = 0.0;
        } else {
            msg.linear.x = 0.2;
        }
    } 
    else if (command == "MOVE_BACKWARD") {
        msg.linear.x = -0.2; // Backing up is usually allowed even if front is blocked
    } 
    else if (command == "ROTATE_LEFT") {
        msg.angular.z = 0.5;
    } 
    else if (command == "ROTATE_RIGHT") {
        msg.angular.z = -0.5;
    } 
    else {
        // STOP
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