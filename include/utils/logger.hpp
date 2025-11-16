#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/async.h>
#include <memory>

class Logger {
public:
    static std::shared_ptr<spdlog::logger>& Get() {
        static std::shared_ptr<spdlog::logger> logger = CreateLogger();
        return logger;
    }

private:
    static std::shared_ptr<spdlog::logger> CreateLogger() {
        // Asynchronous mode: queue size 8192, one background worker
        spdlog::init_thread_pool(8192, 1);

        // Console output (colored)
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        // Rolling file sink (size limit + automatic rotation)
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "logs/app.log",
            5 * 1024 * 1024,  // 5MB
            3                 // Keep 3 rotated log files
        );

        // Combine multiple sinks
        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

        auto logger = std::make_shared<spdlog::async_logger>(
            "AppLogger",
            sinks.begin(),
            sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block
        );

        logger->set_level(spdlog::level::debug); // Adjustable: trace/debug/info
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [T:%t] %v");

        spdlog::register_logger(logger);
        return logger;
    }
};
