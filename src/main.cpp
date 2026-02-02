#include <ctime>
#include <memory>
#include <vector>

#include <zerolancom/zerolancom.hpp>

#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"

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
    std::string group = proxy_reader.getValue<std::string>("group");
    int group_port = proxy_reader.getValue<int>("group_port");
    std::string group_name = proxy_reader.getValue<std::string>("group_name");
    zlc::init(node_name, proxy_ip, group, group_port, group_name);
    zlc::info("Starting Franka Control Proxy with node name: {}", node_name);
    zlc::info("Using proxy IP address: {} at group {}:{} with group name {}", proxy_ip, group, group_port, group_name);

    std::string arm_config_path = proxy_reader.getValue<std::string>("arm_config_path");
    if (arm_config_path.empty())
    {
        zlc::error("Arm config path is empty in proxy config file.");
        return 1;
    }
    FrankaArmProxy robot_proxy(arm_config_path);

    std::string gripper_config_path = proxy_reader.getValue<std::string>("gripper_config_path");
    if (gripper_config_path.empty())
    {
        zlc::error("Gripper config path is empty in proxy config file.");
        return 1;
    }

    FrankaGripperProxy gripper_proxy(proxy_reader.getValue<std::string>("gripper_config_path"));

    zlc::spin();
    return 0;
}
