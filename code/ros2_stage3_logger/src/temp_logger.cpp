// Stage 4: servo temperature logger (ROS 2).
// Reads one bus servo's temperature every interval_sec and appends it to CSV until Ctrl+C.
// (An earlier version also read voltage; that reading was unreliable, so this version logs temperature only.)
#include <chrono>
#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/srv/get_bus_servo_state.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("temp_logger");
  int servo_id    = node->declare_parameter<int>("servo_id", 2);
  double interval = node->declare_parameter<double>("interval_sec", 10.0);
  std::string csv_path = node->declare_parameter<std::string>("csv_path", "/home/ubuntu/temp_run.csv");

  auto client = node->create_client<ros_robot_controller_msgs::srv::GetBusServoState>(
      "/ros_robot_controller/bus_servo/get_state");
  RCLCPP_INFO(node->get_logger(), "waiting for get_state service...");
  client->wait_for_service();

  std::ofstream csv(csv_path);
  csv << "t_sec,temperature_C\n";
  auto t0 = node->now();
  RCLCPP_INFO(node->get_logger(), "logging servo %d temp every %.0fs -> %s (Ctrl+C to stop)",
              servo_id, interval, csv_path.c_str());

  while (rclcpp::ok()) {
    auto req = std::make_shared<ros_robot_controller_msgs::srv::GetBusServoState::Request>();
    ros_robot_controller_msgs::msg::GetBusServoCmd c;
    c.id = servo_id; c.get_temperature = 1;
    req->cmd.push_back(c);
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, fut, 1000ms) == rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = fut.get();
      if (!resp->state.empty() && !resp->state[0].temperature.empty()) {
        double t = (node->now() - t0).seconds();
        int temp = resp->state[0].temperature[0];
        csv << t << "," << temp << "\n";
        csv.flush();
        RCLCPP_INFO(node->get_logger(), "t=%.0fs temp=%dC", t, temp);
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds((int)(interval * 1000)));
  }
  csv.close();
  rclcpp::shutdown();
  return 0;
}
