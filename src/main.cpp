#include <ctime>
#include <memory>
#include <vector>
#include <zerolancom/zerolancom.hpp>

#include "franka_arm_proxy.hpp"

int main(int argc, char** argv)
{
    // check configpath arguments
    if (argc != 2)
    {
        zlc::error("Please provide proxy config path");
        return 1;
    }

    //initialize and start proxies
    std::string proxy_config_path = argv[1];
    ConfigFileReader proxy_reader(proxy_config_path);
    std::string node_name = proxy_reader.getValue<std::string>("node_name");
    std::string proxy_ip = proxy_reader.getValue<std::string>("proxy_ip");
    zlc::info("Starting Franka Control Proxy with node name: {}", node_name);
    zlc::info("Using proxy IP address: {}", proxy_ip);
    zlc::init(node_name, proxy_ip);

    std::string arm_config_path = proxy_reader.getValue<std::string>("arm_config_path");
    if (arm_config_path.empty())
    {
        zlc::error("Arm config path is empty in proxy config file.");
        return 1;
    }
    FrankaArmProxy robot_proxy(arm_config_path);

    // FrankaGripperProxy gripper_proxy(gripper_cfg);
    // gripper_proxy.start();
    zlc::spin();
    return 0;
}
