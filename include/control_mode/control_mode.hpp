#pragma once
#include <control_mode/abstract_control_mode.hpp>
#include <control_mode/hybrid_joint_impedance_control.hpp>
#include <control_mode/idle_control_mode.hpp>
#include <control_mode/gravity_comp_control.hpp>

class ControlModeFactory
{
  public:
    // Register control modes
    static void registerControlModes(
        std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>>& registry,
        FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
        AtomicDoubleBuffer<franka::RobotState>& state_buffer,
        const SafetyLimitConfig& safety_config,
        const std::string& robot_name)
    {
        registry["Idle"] = std::make_unique<IdleControlMode>(safety_config, robot_name);
        registry["GravityComp"] = std::make_unique<GravityCompControl>(safety_config, robot_name);
        registry["HybridJointImpedance"] =
            std::make_unique<HybridJointImpedanceControl>(safety_config, robot_name);
        for (const auto& pair : registry)
        {
            zlc::info("[ControlModeFactory] Registered mode: {}", pair.first);
            pair.second->initController(robot, pinocchio_model, state_buffer);
        }
    }
};
