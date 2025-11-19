#include <ctime>
#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"


int main(int argc, char **argv)
{
    //setup logging
    //build 2 sink for both console and file logging
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    
    std::time_t t = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        std::string("logs/output_") + ts + ".txt", true
    );

    // combine sink
    std::vector<spdlog::sink_ptr> sinks { console_sink, file_sink };
    auto logger = std::make_shared<spdlog::logger>("multi_logger", sinks.begin(), sinks.end());

    // set default logger
    spdlog::set_default_logger(logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    
    //check configpath argument
    if (argc != 2) {
        spdlog::error("Please provide the config path as the sole argument.");
        return 1;
    }
    //initialize and start proxies
    std::string config_path = argv[1];
    FrankaArmProxy robot_proxy(config_path);
    FrankaGripperProxy gripper_proxy(config_path);
    robot_proxy.start();
    gripper_proxy.start();
    robot_proxy.spin();
    return 0;
}
