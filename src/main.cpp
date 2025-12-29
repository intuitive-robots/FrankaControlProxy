#include <ctime>
#include <memory>
#include <vector>
#include <zerolancom/zerolancom.hpp>
#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"
#include "utils/franka_config.hpp"


int main(int argc, char **argv)
{
    
    // check configpath arguments
    if (argc != 3) {
        zlc::error("Please provide two config paths: <arm_config.yaml> <gripper_config.yaml>");
        return 1;
    }
    
    //initialize and start proxies
    std::string proxy_config_path = argv[1];
    ConfigFileReader proxy_reader(proxy_config_path);
    std::string node_name = proxy_reader.getValue<std::string>("node_name", "FrankaControlProxy");
    std::string proxy_ip = proxy_reader.getValue<std::string>("proxy_ip", "");
    zlc::init(node_name, proxy_ip);

    
    FrankaArmProxy robot_proxy(arm_cfg);
    FrankaGripperProxy gripper_proxy(gripper_cfg);
    robot_proxy.start();
    gripper_proxy.start();
    zlc::spin();
    return 0;
}
