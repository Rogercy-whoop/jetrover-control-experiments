// Final stage: whole-vehicle heading PID (ROS 2).
// Drives forward at a fixed speed and corrects heading with angular.z,
// using the yaw from /odom (an EKF estimate that fuses wheel odometry with the IMU).
// kp = 0 gives the open-loop baseline.
#include <chrono>
#include <fstream>
#include <memory>
#include <cmath>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class HeadingPID : public rclcpp::Node {
public:
  HeadingPID() : Node("heading_pid") {
    fwd_     = declare_parameter<double>("forward_speed", 0.1);
    kp_      = declare_parameter<double>("kp", 1.5);
    ki_      = declare_parameter<double>("ki", 0.0);
    kd_      = declare_parameter<double>("kd", 0.2);
    run_sec_ = declare_parameter<double>("run_seconds", 5.0);
    max_w_   = declare_parameter<double>("max_omega", 1.0);
    csv_path_= declare_parameter<std::string>("csv_path", "/home/ubuntu/heading_run.csv");

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&HeadingPID::on_odom, this, std::placeholders::_1));

    csv_.open(csv_path_);
    csv_ << "t_sec,target_yaw,actual_yaw,error_deg,omega_cmd\n";
    RCLCPP_INFO(get_logger(), "waiting for /odom...");
    timer_ = create_wall_timer(20ms, std::bind(&HeadingPID::loop, this));   // 50 Hz
  }

  void stop_robot() {
    geometry_msgs::msg::Twist z;
    pub_->publish(z); pub_->publish(z);
    RCLCPP_INFO(get_logger(), "STOP sent");
  }

private:
  static double yaw_from_quat(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr m) {
    auto& q = m->pose.pose.orientation;
    yaw_ = yaw_from_quat(q.x, q.y, q.z, q.w);
    have_yaw_ = true;
  }

  void loop() {
    if (!have_yaw_) return;
    if (!started_) {
      yaw0_ = yaw_; t0_ = now(); started_ = true;          // lock the starting heading as the target
      RCLCPP_INFO(get_logger(), "target yaw locked, fwd %.2f kp %.2f for %.1fs", fwd_, kp_, run_sec_);
    }
    double t = (now() - t0_).seconds();
    if (t > run_sec_) {
      timer_->cancel(); stop_robot(); csv_.flush(); csv_.close();
      RCLCPP_INFO(get_logger(), "done -> %s", csv_path_.c_str());
      rclcpp::shutdown(); return;
    }
    double e = yaw0_ - yaw_;
    while (e > M_PI) e -= 2 * M_PI;
    while (e < -M_PI) e += 2 * M_PI;
    integ_ += e * 0.02;
    double de = (e - laste_) / 0.02;
    laste_ = e;
    double w = kp_ * e + ki_ * integ_ + kd_ * de;
    if (w > max_w_) w = max_w_;
    if (w < -max_w_) w = -max_w_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = fwd_;
    cmd.angular.z = w;
    pub_->publish(cmd);
    csv_ << t << "," << yaw0_ * 180 / M_PI << "," << yaw_ * 180 / M_PI << ","
         << e * 180 / M_PI << "," << w << "\n";
  }

  double fwd_, kp_, ki_, kd_, run_sec_, max_w_, yaw_ = 0, yaw0_ = 0, integ_ = 0, laste_ = 0;
  std::string csv_path_;
  bool have_yaw_ = false, started_ = false;
  rclcpp::Time t0_;
  std::ofstream csv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<HeadingPID> g_node;
void sigint(int) { if (g_node) g_node->stop_robot(); rclcpp::shutdown(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<HeadingPID>();
  std::signal(SIGINT, sigint);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
