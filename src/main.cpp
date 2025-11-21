#include <ctime>
#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "utils/logger.hpp"
#include "utils/franka_config.hpp"
#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"


int main(int argc, char **argv)
{
    //initialize logger
    utils::Logger::init(false);//true to enable file logging
    utils::Logger::setLevel(utils::LogLevel::INFO);




    //check configpath argument
    if (argc != 2) {
        LOG_ERROR("Please provide the config path as the sole argument.");
        return 1;
    }
    //initialize and start proxies
    std::string config_path = argv[1];
    FrankaConfig config(config_path);
    const auto& cfg = config.data();
    FrankaArmProxy robot_proxy(cfg);
    FrankaGripperProxy gripper_proxy(cfg);
    robot_proxy.start();
    gripper_proxy.start();
    robot_proxy.spin();
    return 0;
}
