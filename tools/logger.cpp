#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools {
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger(spdlog::level::level_enum level) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(level);

#ifndef NDEBUG
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log",
                               std::chrono::system_clock::now());
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(level);

  auto target = spdlog::sinks_init_list{file_sink, console_sink};
#else
  auto target = spdlog::sinks_init_list{console_sink};
#endif

  logger_ = std::make_shared<spdlog::logger>("", target);
  logger_->set_level(level);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger() {
  if (!logger_)
    set_logger(spdlog::level::debug);
  return logger_;
}

} // namespace tools
