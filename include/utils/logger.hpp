#pragma once

#include <memory>
#include <string>
#include <vector>
#include <ctime>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>
#include <spdlog/async_logger.h>

namespace utils {

// Log levels for unified control
enum class LogLevel {
    TRACE = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    // Initialize async logging system.
    // enable_file_logging = true → console + file
    // enable_file_logging = false → console only
    static void init(bool enable_file_logging = false,
                     const std::string& log_dir = "logs")
    {
        // Create async thread pool (1 thread, 8192 queue size)
        spdlog::init_thread_pool(8192, 1);

        std::vector<spdlog::sink_ptr> sinks;

        // Console sink
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sinks.push_back(console_sink);

        // Optional file sink
        if (enable_file_logging) {
            std::filesystem::create_directories(log_dir);

            std::time_t t = std::time(nullptr);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));

            std::string file_path = log_dir + "/output_" + ts + ".txt";

            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                file_path, true
            );
            sinks.push_back(file_sink);
        }

        // Async logger using the global thread pool
        auto async_logger = std::make_shared<spdlog::async_logger>(
            "proxy_async_logger",
            sinks.begin(),
            sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block
        );

        // Register and set as default logger
        spdlog::set_default_logger(async_logger);

        // Log formatting
        // if want to see thread id:→ [%Y-%m-%d %H:%M:%S.%e] [T%t] [%^%l%$] %v
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    }

    // Set global logging level
    static void setLevel(LogLevel lvl) {
        spdlog::level::level_enum spd_lvl = toSpdLevel(lvl);
        spdlog::set_level(spd_lvl);
    }

private:
    static spdlog::level::level_enum toSpdLevel(LogLevel lvl) {
        switch (lvl) {
            case LogLevel::TRACE: return spdlog::level::trace;
            case LogLevel::INFO:  return spdlog::level::info;
            case LogLevel::WARN:  return spdlog::level::warn;
            case LogLevel::ERROR: return spdlog::level::err;
            case LogLevel::FATAL: return spdlog::level::critical;
        }
        return spdlog::level::info;
    }
};

} // namespace utils

// Logging macros using async logger (supports {} format)
#define LOG_TRACE(...) spdlog::trace(__VA_ARGS__)
#define LOG_INFO(...)  spdlog::info(__VA_ARGS__)
#define LOG_WARN(...)  spdlog::warn(__VA_ARGS__)
#define LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define LOG_FATAL(...) spdlog::critical(__VA_ARGS__)
