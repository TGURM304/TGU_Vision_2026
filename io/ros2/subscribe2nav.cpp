#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber")
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    std::bind(&Subscribe2Nav::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nav_x_ = msg->linear.x;
  nav_y_ = msg->linear.y;
  nav_w_ = msg->angular.z;

  // RCLCPP_INFO(
  //   this->get_logger(),
  //   "Nav cmd_vel: vx=%.3f vy=%.3f wz=%.3f",
  //   nav_x_, nav_y_, nav_w_);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node starting...");
  rclcpp::spin(this->shared_from_this());
}

std::tuple<double, double, double> Subscribe2Nav::get_nav_cmd()
{
  return {nav_x_, nav_y_, nav_w_};
}

}  // namespace io