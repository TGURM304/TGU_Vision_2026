#include "action2nav.hpp"

#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace io
{

// ===================== 内部实现 =====================
struct Action2Nav::Impl : public rclcpp::Node
{
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  Impl()
  : Node("action2nav_node")
  {
    client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");

    RCLCPP_INFO(this->get_logger(), "action2nav node initialized.");
  }

  ~Impl()
  {
    RCLCPP_INFO(this->get_logger(), "action2nav node shutting down.");
  }

  void start()
  {
    RCLCPP_INFO(this->get_logger(), "action2nav node starting...");
    // rclcpp::spin(this->shared_from_this());
    rclcpp::spin(this->get_node_base_interface());
  }

  bool send_path(const std::vector<std::tuple<double, double, double>>& poses)
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
      return false;
    }

    NavigateThroughPoses::Goal goal_msg;

    total_poses_ = poses.size();
    remaining_poses_ = poses.size();
    status_ = NavStatus::RUNNING;

    for (const auto& [x, y, yaw] : poses) {
      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = "map";
      pose.header.stamp = this->now();

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);

      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      goal_msg.poses.push_back(pose);
    }

    auto options =
      rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

    // ===== feedback =====
    options.feedback_callback =
      [this](
        GoalHandleNavThroughPoses::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
      {
        remaining_poses_ = feedback->number_of_poses_remaining;

        RCLCPP_INFO(
          this->get_logger(),
          "Remaining poses: %zu",
          remaining_poses_);
      };

    // ===== result =====
    options.result_callback =
      [this](const GoalHandleNavThroughPoses::WrappedResult& result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            status_ = NavStatus::SUCCEEDED;
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            break;

          case rclcpp_action::ResultCode::ABORTED:
            status_ = NavStatus::FAILED;
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            break;

          case rclcpp_action::ResultCode::CANCELED:
            status_ = NavStatus::CANCELED;
            RCLCPP_WARN(this->get_logger(), "Navigation canceled");
            break;

          default:
            status_ = NavStatus::FAILED;
            break;
        }
      };

    client_->async_send_goal(goal_msg, options);

    RCLCPP_INFO(this->get_logger(), "Path sent to Nav2.");

    return true;
  }

  NavStatus get_status()
  {
    return status_;
  }

  std::size_t get_remaining_poses()
  {
    return remaining_poses_;
  }

  // ===================== 成员 =====================
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;

  NavStatus status_{NavStatus::IDLE};
  std::size_t remaining_poses_{0};
  std::size_t total_poses_{0};
};

// ===================== 外部封装 =====================
Action2Nav::Action2Nav()
{
  impl_ = new Impl();
}

Action2Nav::~Action2Nav()
{
  delete impl_;
}

void Action2Nav::start()
{
  impl_->start();
}

bool Action2Nav::send_path(
  const std::vector<std::tuple<double, double, double>>& poses)
{
  return impl_->send_path(poses);
}

NavStatus Action2Nav::get_status()
{
  return impl_->get_status();
}

std::size_t Action2Nav::get_remaining_poses()
{
  return impl_->get_remaining_poses();
}

}  // namespace io