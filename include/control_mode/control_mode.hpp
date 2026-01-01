#pragma once
#include <control_mode/abstract_control_mode.hpp>
#include <control_mode/idle_control_mode.hpp>
#include <control_mode/cartesian_pose_mode.hpp>
#include <control_mode/cartesian_velocity_mode.hpp>
#include <control_mode/joint_position_mode.hpp>
#include <control_mode/joint_velocity_mode.hpp>
#include <control_mode/human_control_mode.hpp>

class ControlModeFactory {
public:

    // static void registerMode(const std::string& name, std::function<std::shared_ptr<AbstractControlMode>()> creator) {
    //     zlc::info("[ControlModeFactory] Registering mode: {}", name);
    //     getRegistry()[name] = std::move(creator);
    // }

    // static std::shared_ptr<AbstractControlMode> create(const ControlModeID id) {
    //     auto& reg = getRegistry();
    //     std::string name = protocol::toString(id);
    //     if (auto it = reg.find(name); it != reg.end())
    //         return it->second();
    //     throw std::runtime_error("Unknown mode: " + name);
    // }

    // Register control modes
    static void registerControlModes(std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>>& registry) {
        // registry["IDLE"] = std::make_unique<IdleControlMode>();
        // registry["CARTESIAN_POSE"] = std::make_unique<CartesianPoseMode>();
        registry["CARTESIAN_VELOCITY"] = std::make_unique<CartesianVelocityMode>();
        // registry["JOINT_POSITION"] = std::make_unique<JointPositionMode>();
        // registry["JOINT_VELOCITY"] = std::make_unique<JointVelocityMode>();
        // registry["HUMAN_CONTROL"] = std::make_unique<HumanControlMode>();
        for (const auto& pair : registry) {
            zlc::info("[ControlModeFactory] Registered mode: {}", pair.first);
        }
    }

};
