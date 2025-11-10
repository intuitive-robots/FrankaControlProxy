#include "franka_arm_proxy.hpp"
#include <iostream>


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }

    std::string config_path = argv[1];
    FrankaArmProxy proxy(config_path);
    std::string type = proxy.getType();
    std::cout << "[INFO] FrankaProxy initialized with type: " << type << std::endl;
    std::cout<<"go start!"<<std::endl;
    proxy.start();
    proxy.spin();
    
    return 0;
}
