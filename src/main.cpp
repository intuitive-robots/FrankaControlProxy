#include <ctime>
#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <zerolancom/zerolancom.hpp>
#include "utils/logger.hpp"
#include "utils/franka_config.hpp"
#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"


int main(int argc, char **argv)
{
    // initialize logger
    utils::Logger::init(false);//true to enable file logging
    utils::Logger::setLevel(utils::LogLevel::INFO);
    zerolancom::ZeroLanComNode& node = zerolancom::ZeroLanComNode::init("Franka", "127.0.0.1");
    
    // check configpath arguments
    if (argc != 3) {
        LOG_ERROR("Please provide two config paths: <arm_config.yaml> <gripper_config.yaml>");
        return 1;
    }
    
    //initialize and start proxies
    std::string arm_config_path = argv[1];
    std::string gripper_config_path = argv[2];
    FrankaConfig config;
    config.loadFromFiles(arm_config_path, gripper_config_path);
    
    const auto& arm_cfg = config.armData();
    const auto& gripper_cfg = config.gripperData();
    
    FrankaArmProxy robot_proxy(arm_cfg, node);
    FrankaGripperProxy gripper_proxy(gripper_cfg, node);
    robot_proxy.start();
    gripper_proxy.start();
    robot_proxy.spin();
    return 0;
}
