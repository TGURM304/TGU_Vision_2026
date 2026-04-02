#pragma once

#include <tuple>
#include <vector>
#include <cstddef>

namespace io
{

enum class NavStatus
{
  IDLE = 0,
  RUNNING,
  SUCCEEDED,
  FAILED,
  CANCELED
};

class Action2Nav
{
public:
  Action2Nav();
  ~Action2Nav();

  void start();

  bool send_path(const std::vector<std::tuple<double, double, double>>& poses);

  NavStatus get_status();
  std::size_t get_remaining_poses();

private:
  struct Impl;
  Impl* impl_;   // PImpl，避免头文件依赖 ROS2
};

}  // namespace io