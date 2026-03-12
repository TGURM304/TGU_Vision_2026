#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

#define RAD (180.f / M_PI)

const std::string keys =
    "{help h usage ? | | 输出命令行参数说明}"
    "{@config-path   | ./configs/standard.yaml | yaml配置文件路径 }";

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

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  // auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // auto_buff::Buff_Detector buff_detector(config_path);
  // auto_buff::Solver buff_solver(config_path);
  // auto_buff::SmallTarget buff_small_target;
  // auto_buff::BigTarget buff_big_target;
  // auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto yaml = YAML::LoadFile(config_path);
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto R_camera2gimbal =
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target_ = target_queue.front();

        if (!target_.has_value()) {
          gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
          continue;
        }
        auto target = *target_;

        // 找到视图上平面距离最小的装甲板作为目标
        Eigen::Vector3d xyz;
        auto min_dist = 1e10;
        for (auto &xyza : target.armor_xyza_list()) {
          auto dist = xyza.head<2>().norm();
          if (dist < min_dist) {
            min_dist = dist;
            xyz = xyza.head<3>();
          }
        }
        tools::logger()->debug(
            "target vector in camera: ({:.3f},{:.3f},{:.3f})", xyz.x(), xyz.y(),
            xyz.z());

        // 计算欧拉角
        // xyz 是在世界坐标系下的向量
        double yaw = atan2(xyz.y(), xyz.x());
        double pitch = -atan2(xyz.z(), hypot(xyz.x(), xyz.y()));
        if (yaw > M_PI_2)
          yaw -= M_PI;
        if (yaw < -M_PI_2)
          yaw += M_PI;

        gimbal.send(true, true, yaw, 0, 0, pitch, 0, 0);
        tools::logger()->debug("planning: yaw {:.3f}, pitch {:.3f}", yaw * RAD,
                               pitch * RAD);

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

  while (!exiter.exit()) {
    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
    }

    camera.read(img, t);
    // tools::logger()->debug("image size {}x{}", img.size[0], img.size[1]);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    // recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto armors = yolo.detect(img);
      auto targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    } else
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  }

  quit = true;
  if (plan_thread.joinable())
    plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}