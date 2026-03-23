```cpp
#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <memory>
#include <thread>
#include <tuple>

#include "subscribe2nav.hpp"

namespace io
{

class ROS2
{
public:
  ROS2();
  ~ROS2();

  // 获取 Nav2 给出的底盘控制 (vx, vy, wz)
  std::tuple<double, double, double> get_nav_cmd();

private:
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;

  std::unique_ptr<std::thread> subscribe_spin_thread_;
};

}  // namespace io

#endif
```
