#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include <spdlog/spdlog.h>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal_nav.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/crc.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

#include "tools/debug.hpp"

const std::string keys =
    "{help h usage ? | | 输出命令行参数说明}"
    "{@config-path   | ./configs/sentry.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::GimbalNav gimbal(config_path);
  io::Camera camera(config_path);
  io::ROS2 ros2;

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  io::VisionToGimbal vision2gimbal;

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    
    // 自瞄区配置
    Eigen::Vector4d xyza = {};

    // 导航区配置
    enum class TargetPoint{
      START_POINT ,
      SUPPLY_POINT,
      POINT1,
      POINT2,
      POINT3
    };
  
    struct Point{
      float x;
      float y;
    };

    std::map<TargetPoint, Point> point_map = {
      {TargetPoint::START_POINT,  {1.8, 1.3}},
      {TargetPoint::SUPPLY_POINT, {1.2, 1.1}},
      {TargetPoint::POINT1,       {4.3, 6.3}},
      {TargetPoint::POINT2,       {5.3, 5.5}},
      {TargetPoint::POINT3,       {5.3, 7.2}}
    };

    TargetPoint last_point = TargetPoint::START_POINT;
 
    std::vector<TargetPoint> center_path = {
      TargetPoint::POINT1,
      TargetPoint::POINT2,
      TargetPoint::POINT3
    };

    io::NavStatus nav_status = io::NavStatus::IDLE;

    int index = 0;

    bool is_dead = false;
    bool is_center = false;

    Point current_point{0, 0};
    
    while (!quit) {
      auto [anav_x, anav_y, anav_z] = ros2.get_nav_cmd();

      // 自瞄行为区
      if (!target_queue.empty()) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        vision2gimbal.mode = plan.control ? (plan.fire ? 2 : 1) : 0;

        vision2gimbal.yaw = plan.yaw;
        vision2gimbal.yaw_vel = plan.yaw_vel;
        vision2gimbal.yaw_acc = plan.yaw_acc;

        vision2gimbal.pitch = plan.pitch;
        vision2gimbal.pitch_vel = plan.pitch_vel;
        vision2gimbal.pitch_acc = plan.pitch_acc;
      }else{
        vision2gimbal.mode = 0;
        vision2gimbal.yaw = vision2gimbal.yaw_vel = vision2gimbal.yaw_acc = 0;
        vision2gimbal.pitch = vision2gimbal.pitch_vel = vision2gimbal.pitch_acc = 0;
      }


      // 导航行为区
      // 行为树
      /*

      ******************************
      *                            *
      *                            *
      *                            *
      *         ********************
      *         ********************
      *         ********************
      *                 4          *
      *                            *
      *           2                *
      *                            *
      *                 3     B    *
      *                            *
      ********************         *
      ********************         *
      ********************         *
      *                            *
      *------                 A    *
      *  5  1                      *
      ******************************
      坐标：
        1: 起始点 (1.8, 1.3)
        2: 点位1 (4.3, 6.3)
        3: 点位2 (5.3, 5.5)
        4: 点位3 (5.3, 7.2)
        5: 补给点 (1.2, 1.1)

      行为树设计：
        比赛开始?
          |-> 否 -> 待机
          |-> 是 -> 血量充足?
                      |-> 否 -> 回家
                      |-> 是 ->       
      */

      vision2gimbal.nav_x = anav_x;
      vision2gimbal.nav_y = anav_y;
      vision2gimbal.nav_z = anav_z;

      DEBUG("导航状态: {}", (int)nav_status);
      DEBUG("导航向量: {}, {}", (float)vision2gimbal.nav_x, (float)vision2gimbal.nav_y);
      
      // 比赛开始
      if(gimbal.state().is_start == 4){
        // 血量判断
        if(gimbal.state().hp > 100 && !is_dead){ // 血量充足
          // 定时换位
          // if (nav_status == io::NavStatus::SUCCEEDED &&
          //   last_status != io::NavStatus::SUCCEEDED) {

          //   auto elapsed = std::chrono::steady_clock::now() - t0;
          //   if(elapsed > 5s){
          //   t0 = std::chrono::steady_clock::now();

          //   index = (index + 1) % center_path.size();
          //   last_point = center_path[index];
          //   current_point = point_map.at(last_point);
          //   }
            // DEBUG("cnm");

          if(nav_status == io::NavStatus::SUCCEEDED){
              is_center = true;
          }
          current_point = point_map.at(TargetPoint::POINT3);
          std::vector<std::tuple<double, double, double>> current_path = {{current_point.x, current_point.y,0}};
          if (nav_status == io::NavStatus::FAILED || nav_status == io::NavStatus::CANCELED || !is_center) {
            ros2.send_path(current_path);
            DEBUG("发路径: {}, {}", current_point.x, current_point.y);
          }
        }else if(gimbal.state().hp <= 100 || is_dead){ // 回家
          // 设置死亡标志位
          is_dead = true;
          // 在家直到回到满血
          current_point = point_map.at(TargetPoint::START_POINT);
          std::vector<std::tuple<double, double, double>> current_path = {{current_point.x, current_point.y,0}};

          if (nav_status == io::NavStatus::FAILED || nav_status == io::NavStatus::CANCELED || is_center) {
            ros2.send_path(current_path);
            DEBUG("发路径: {}, {}", current_point.x, current_point.y);

          }

          if(nav_status == io::NavStatus::SUCCEEDED){
              is_center = false;
          }

          if(gimbal.state().hp >= 400){
            is_dead = false;
          }

        }else{ // 神秘状态
          DEBUG("进这个是傻逼");
        }
        vision2gimbal.nav_x = anav_x;
        vision2gimbal.nav_y = anav_y;
        DEBUG("cnm");
      }else{
        is_dead = false;
        current_point = point_map.at(TargetPoint::START_POINT);
        vision2gimbal.mode = 0;
        vision2gimbal.nav_x = 0;
        vision2gimbal.nav_y = 0;
        vision2gimbal.nav_z = 0;
      }

      // DEBUG("血量：{} ; 比赛状态：{}",gimbal.state().hp, gimbal.state().is_start);
      gimbal.send(vision2gimbal);
      DEBUG("{}", (float)vision2gimbal.nav_x);
      std::this_thread::sleep_for(5ms);
    }
  });

  while (!exiter.exit()) {
    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
    }

    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    // recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);
  }

  quit = true;
  if (plan_thread.joinable())
    plan_thread.join();
  gimbal.send(io::VisionToGimbal{});

  return 0;
}