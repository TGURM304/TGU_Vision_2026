```cpp
#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tuple>

namespace io
{

class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();
  ~Subscribe2Nav();

  void start();

  // 获取导航给出的底盘控制 (vx, vy, wz)
  std::tuple<double, double, double> get_nav_cmd();

private:
  // /cmd_vel 回调
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // ROS2 subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // 缓存的导航控制量
  double nav_x_;
  double nav_y_;
  double nav_w_;
};

}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
```
