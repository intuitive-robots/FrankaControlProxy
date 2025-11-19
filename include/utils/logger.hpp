#pragma once
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace utils {

enum class LogLevel {
    TRACE = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& instance() {
        static Logger instance;
        return instance;
    }

    void setLevel(LogLevel level) {
        level_ = level;
    }

    void setOutputFile(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        file_logger_ = spdlog::basic_logger_mt("utils_file_logger", path, true);
    }

    void enableColor(bool /*enable*/ = true) {
        // color formatting handled by spdlog's default logger pattern
    }

    template <typename... Args>
    void log(LogLevel level, Args&&... args) {
        if (level < level_) return;

        std::ostringstream oss;
        (oss << ... << args);

        const auto spd_level = toSpdLevel(level);
        if (file_logger_) {
            file_logger_->log(spd_level, oss.str());
        } else {
            spdlog::log(spd_level, oss.str());
        }
    }

    template <typename... Args> void trace(Args&&... a) { log(LogLevel::TRACE, std::forward<Args>(a)...); }
    template <typename... Args> void info(Args&&... a)  { log(LogLevel::INFO,  std::forward<Args>(a)...); }
    template <typename... Args> void warn(Args&&... a)  { log(LogLevel::WARN,  std::forward<Args>(a)...); }
    template <typename... Args> void error(Args&&... a) { log(LogLevel::ERROR, std::forward<Args>(a)...); }
    template <typename... Args> void fatal(Args&&... a) { log(LogLevel::FATAL, std::forward<Args>(a)...); }

private:
    Logger() = default;

    static spdlog::level::level_enum toSpdLevel(LogLevel level) {
        switch (level) {
            case LogLevel::TRACE: return spdlog::level::trace;
            case LogLevel::INFO:  return spdlog::level::info;
            case LogLevel::WARN:  return spdlog::level::warn;
            case LogLevel::ERROR: return spdlog::level::err;
            case LogLevel::FATAL: return spdlog::level::critical;
        }
        return spdlog::level::info;
    }

    LogLevel level_ = LogLevel::TRACE;
    std::shared_ptr<spdlog::logger> file_logger_;
    std::mutex mutex_;
};

} // namespace utils

#define LOG_TRACE(...) utils::Logger::instance().trace(__VA_ARGS__)
#define LOG_INFO(...)  utils::Logger::instance().info(__VA_ARGS__)
#define LOG_WARN(...)  utils::Logger::instance().warn(__VA_ARGS__)
#define LOG_ERROR(...) utils::Logger::instance().error(__VA_ARGS__)
#define LOG_FATAL(...) utils::Logger::instance().fatal(__VA_ARGS__)
