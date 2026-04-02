#include "ros2.hpp"

namespace io
{

ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  // ===== 初始化模块 =====
  subscribe2nav_ = std::make_shared<Subscribe2Nav>();
  action2nav_ = std::make_shared<Action2Nav>();

  // ===== 启动线程 =====
  subscribe_spin_thread_ =
    std::make_unique<std::thread>(
      [this]() { subscribe2nav_->start(); });

  action_spin_thread_ =
    std::make_unique<std::thread>(
      [this]() { action2nav_->start(); });
}

ROS2::~ROS2()
{
  if (subscribe_spin_thread_ && subscribe_spin_thread_->joinable()) {
    subscribe_spin_thread_->join();
  }

  if (action_spin_thread_ && action_spin_thread_->joinable()) {
    action_spin_thread_->join();
  }

  rclcpp::shutdown();
}

// ===================== 原接口 =====================
std::tuple<double, double, double> ROS2::get_nav_cmd()
{
  return subscribe2nav_->get_nav_cmd();
}

// ===================== 新增：发送路径 =====================
bool ROS2::send_path(
  const std::vector<std::tuple<double, double, double>>& poses)
{
  return action2nav_->send_path(poses);
}

// ===================== 新增：状态 =====================
NavStatus ROS2::get_nav_status()
{
  return action2nav_->get_status();
}

// ===================== 新增：剩余点 =====================
std::size_t ROS2::get_remaining_poses()
{
  return action2nav_->get_remaining_poses();
}

}  // namespace io