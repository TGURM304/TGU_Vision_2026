#pragma once

#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include "subscribe2nav.hpp"
#include "action2nav.hpp"

namespace io
{

class ROS2
{
public:
  ROS2();
  ~ROS2();

  // ===== 原有接口 =====
  std::tuple<double, double, double> get_nav_cmd();

  // ===== 新增：导航接口 =====
  bool send_path(const std::vector<std::tuple<double, double, double>>& poses);

  NavStatus get_nav_status();

  std::size_t get_remaining_poses();

private:
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;
  std::shared_ptr<Action2Nav> action2nav_;

  std::unique_ptr<std::thread> subscribe_spin_thread_;
  std::unique_ptr<std::thread> action_spin_thread_;
};

}  // namespace io