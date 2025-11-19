#include <iostream>
#include "franka_arm_proxy.hpp"
#include "franka_gripper_proxy.hpp"


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }

    std::string config_path = argv[1];
    FrankaArmProxy robot_proxy(config_path);
    FrankaGripperProxy gripper_proxy(config_path);
    robot_proxy.start();
    gripper_proxy.start();
    robot_proxy.spin();
    return 0;
}
