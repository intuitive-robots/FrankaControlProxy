#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <string>
#include <thread>

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
        if (file_.is_open()) file_.close();
        file_.open(path, std::ios::out | std::ios::app);
    }

    void enableColor(bool enable = true) {
        use_color_ = enable;
    }

    template <typename... Args>
    void log(LogLevel level, Args&&... args) {
        if (level < level_) return; // filtered out

        std::ostringstream oss;
        (oss << ... << args);  // fold expression (C++17)

        std::lock_guard<std::mutex> lock(mutex_);
        std::string output = format(level, oss.str());

        if (file_.is_open())
            file_ << output << std::endl;
        else
            std::cout << output << std::endl;
    }

    // Helper macros for convenience
    template <typename... Args> void trace(Args&&... a) { log(LogLevel::TRACE, std::forward<Args>(a)...); }
    template <typename... Args> void info(Args&&... a)  { log(LogLevel::INFO,  std::forward<Args>(a)...); }
    template <typename... Args> void warn(Args&&... a)  { log(LogLevel::WARN,  std::forward<Args>(a)...); }
    template <typename... Args> void error(Args&&... a) { log(LogLevel::ERROR, std::forward<Args>(a)...); }
    template <typename... Args> void fatal(Args&&... a) { log(LogLevel::FATAL, std::forward<Args>(a)...); }

private:
    Logger() = default;
    ~Logger() { if (file_.is_open()) file_.close(); }

    std::string format(LogLevel level, const std::string& msg) {
        // Timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
    #ifdef _WIN32
        localtime_s(&tm, &t);
    #else
        localtime_r(&t, &tm);
    #endif

        std::ostringstream oss;
        oss << std::put_time(&tm, "%H:%M:%S");

        // Thread ID
        oss << " [" << std::this_thread::get_id() << "] ";

        // Level
        if (use_color_)
            oss << levelColor(level) << levelName(level) << "\033[0m";
        else
            oss << levelName(level);

        oss << " : " << msg;
        return oss.str();
    }

    std::string levelName(LogLevel level) const {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::INFO:  return "INFO ";
            case LogLevel::WARN:  return "WARN ";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
        }
        return "UNKWN";
    }

    std::string levelColor(LogLevel level) const {
        switch (level) {
            case LogLevel::TRACE: return "\033[37m";  // white
            case LogLevel::INFO:  return "\033[36m";  // cyan
            case LogLevel::WARN:  return "\033[33m";  // yellow
            case LogLevel::ERROR: return "\033[31m";  // red
            case LogLevel::FATAL: return "\033[41m";  // red background
        }
        return "";
    }

    LogLevel level_ = LogLevel::TRACE;
    std::ofstream file_;
    bool use_color_ = true;
    mutable std::mutex mutex_;
};

} // namespace utils

// Convenience macros
#define LOG_TRACE(...) utils::Logger::instance().trace(__VA_ARGS__)
#define LOG_INFO(...)  utils::Logger::instance().info(__VA_ARGS__)
#define LOG_WARN(...)  utils::Logger::instance().warn(__VA_ARGS__)
#define LOG_ERROR(...) utils::Logger::instance().error(__VA_ARGS__)
#define LOG_FATAL(...) utils::Logger::instance().fatal(__VA_ARGS__)
