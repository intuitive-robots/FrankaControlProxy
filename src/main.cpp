#include "FrankaProxy.hpp"
#include "control_mode_registry.h"
#include <iostream>


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }

    std::string config_path = argv[1];
    FrankaProxy proxy(config_path);
    registerAllControlModes(proxy);
    std::string type = proxy.getType();
    std::cout << "[INFO] FrankaProxy initialized with type: " << type << std::endl;
    std::cout<<"go start!"<<std::endl;
    proxy.start();
    //proxy.setControlMode("zero_torque");
    proxy.spin();
    
    return 0;
}
