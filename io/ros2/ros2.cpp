#include "ros2.hpp"

namespace io
{

ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  subscribe_spin_thread_ =
    std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  subscribe_spin_thread_->join();

  rclcpp::shutdown();
}

std::tuple<double, double, double> ROS2::get_nav_cmd()
{
  return subscribe2nav_->get_nav_cmd();
}

}  // namespace io