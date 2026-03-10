#include <ctime>
#include <memory>
#include <vector>
#include <zerolancom/zerolancom.hpp>

#include "robots/panda_arm.hpp"
#include "robots/panda_gripper.hpp"
#include "robots/robotiq_gripper.hpp"

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
    YAML::Node robot_node = proxy_reader.getSubNode("robot");
    std::vector<std::unique_ptr<PandaArm>> arms;
    if (robot_node && robot_node.IsSequence())
    {
        for (size_t i = 0; i < robot_node.size(); ++i)
        {
            std::string type = robot_node[i]["type"].as<std::string>();
            std::string cfg = robot_node[i]["config_path"].as<std::string>();
            zlc::info("Robot [{}]: type={}, path={}", i, type, cfg);
            try
            {
                if (type == "panda")
                {
                    arms.push_back(std::make_unique<PandaArm>(cfg));
                }
                else
                {
                    zlc::warn("Unknown robot type: {}", type);
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return 1;
            }
        }
    }

    YAML::Node gripper_node = proxy_reader.getSubNode("grippers");
    std::vector<std::unique_ptr<PandaGripper>> franka_grippers;
    std::vector<std::unique_ptr<RobotiqGripper>> robotiq_grippers;
    if (gripper_node && gripper_node.IsSequence())
    {
        for (const auto& item : gripper_node)
        {
            std::string type = item["type"].as<std::string>();
            std::string cfg = item["config_path"].as<std::string>();
            zlc::info("Gripper: type={}, path={}", type, cfg);
            try
            {
                if (type == "franka_gripper")
                {
                    franka_grippers.push_back(std::make_unique<PandaGripper>(cfg));
                }
                else if (type == "robotiq_gripper")
                {
                    robotiq_grippers.push_back(std::make_unique<RobotiqGripper>(cfg));
                }
                else
                {
                    zlc::warn("Unknown gripper type: {}", type);
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return 1;
            }
        }
    }
    zlc::spin();
    return 0;
}
