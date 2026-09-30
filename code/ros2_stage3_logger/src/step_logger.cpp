// Stage 3: servo step-response logger (ROS 2).
// Fires one position step on a bus servo and logs target vs actual angle at ~50 Hz to CSV.
#include <chrono>
#include <fstream>
#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/servos_position.hpp"
#include "ros_robot_controller_msgs/msg/servo_position.hpp"
#include "ros_robot_controller_msgs/srv/get_bus_servo_state.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("step_logger");

  int servo_id     = node->declare_parameter<int>("servo_id", 2);
  int start_raw    = node->declare_parameter<int>("start_raw", 700);
  int delta_raw    = node->declare_parameter<int>("delta_raw", -250);   // -250 units = -60 deg
  double move_dur  = node->declare_parameter<double>("move_duration", 0.3);
  double record_sec= node->declare_parameter<double>("record_seconds", 6.0);
  std::string csv_path = node->declare_parameter<std::string>("csv_path", "/home/ubuntu/stage3_run.csv");
  int target_raw = start_raw + delta_raw;

  auto pub = node->create_publisher<ros_robot_controller_msgs::msg::ServosPosition>(
      "/ros_robot_controller/bus_servo/set_position", 10);
  auto client = node->create_client<ros_robot_controller_msgs::srv::GetBusServoState>(
      "/ros_robot_controller/bus_servo/get_state");

  RCLCPP_INFO(node->get_logger(), "waiting for get_state service...");
  client->wait_for_service();

  auto read_pos = [&]() -> int {
    auto req = std::make_shared<ros_robot_controller_msgs::srv::GetBusServoState::Request>();
    ros_robot_controller_msgs::msg::GetBusServoCmd c;
    c.id = servo_id; c.get_position = 1;
    req->cmd.push_back(c);
    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, fut, 100ms) == rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = fut.get();
      if (!resp->state.empty() && !resp->state[0].position.empty())
        return resp->state[0].position[0];
    }
    return -1;
  };

  int start_actual = read_pos();
  RCLCPP_INFO(node->get_logger(), "servo %d: %d -> %d (%.1f deg), start actual raw = %d",
              servo_id, start_raw, target_raw, delta_raw * 0.24, start_actual);

  std::ofstream csv(csv_path);
  csv << "t_sec,target_deg_rel,actual_deg_rel,actual_raw,target_raw\n";

  // fire the step
  ros_robot_controller_msgs::msg::ServosPosition cmd;
  cmd.duration = move_dur;
  ros_robot_controller_msgs::msg::ServoPosition sp;
  sp.id = servo_id; sp.position = target_raw;
  cmd.position.push_back(sp);
  pub->publish(cmd);
  auto t0 = node->now();
  RCLCPP_INFO(node->get_logger(), "STEP fired, logging %.1fs", record_sec);

  rclcpp::Rate rate(50);   // 50 Hz
  while (rclcpp::ok()) {
    double t = (node->now() - t0).seconds();
    if (t > record_sec) break;
    int raw = read_pos();
    if (raw >= 0) {
      double actual_deg_rel = (raw - start_actual) * 0.24;   // 0.24 deg per servo unit
      double target_deg_rel = delta_raw * 0.24;
      csv << t << "," << target_deg_rel << "," << actual_deg_rel << "," << raw << "," << target_raw << "\n";
    }
    rate.sleep();
  }
  csv.flush(); csv.close();
  RCLCPP_INFO(node->get_logger(), "done -> %s", csv_path.c_str());
  rclcpp::shutdown();
  return 0;
}
