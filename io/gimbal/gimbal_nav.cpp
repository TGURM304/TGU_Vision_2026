#include "gimbal_nav.hpp"

#include <spdlog/spdlog.h>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
GimbalNav::GimbalNav(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  auto com_baudrate = tools::read<long long>(yaml, "com_baudrate");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(com_baudrate);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&GimbalNav::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

GimbalNav::~GimbalNav()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode GimbalNav::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState GimbalNav::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string GimbalNav::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond GimbalNav::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void GimbalNav::send(io::VisionToGimbal vison2gimbal)
{
  tx_data_.mode = vison2gimbal.mode;
  tx_data_.yaw = vison2gimbal.yaw;
  tx_data_.yaw_vel = vison2gimbal.yaw_vel;
  tx_data_.yaw_acc = vison2gimbal.yaw_acc;
  tx_data_.pitch = vison2gimbal.pitch;
  tx_data_.pitch_vel = vison2gimbal.pitch_vel;
  tx_data_.pitch_acc = vison2gimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool GimbalNav::read(uint8_t * buffer, size_t size)
{
    size_t total = 0;
    auto start = std::chrono::steady_clock::now();
    while (total < size) {
        int sz = serial_.read(buffer + total, size - total);
        if (sz <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 避免死循环
        } else {
            total += sz;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 100) {
            return false; // 超时
        }
    }
    return true;
}

void GimbalNav::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }
    
    // std::cout<< rx_data_.head[0] << ";" << rx_data_.head[1] << std::endl;

    if (rx_data_.head[0] != 'T' || rx_data_.head[1] != 'G') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    // state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void GimbalNav::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io