#pragma once
#include <control_mode/abstract_control_mode.hpp>
#include <control_mode/cartesian_impedance.hpp>
#include <control_mode/gravity_comp_control.hpp>
#include <control_mode/human_control.hpp>
#include <control_mode/hybrid_joint_impedance_control.hpp>
#include <control_mode/idle_control_mode.hpp>
#include <control_mode/osc_control.hpp>

#include "protocol/control_command.hpp"
#include "utils/Pose.h"

class ControlModeFactory
{
  public:
    ControlModeFactory(const std::string& robot_name)
    {
        safety_config_.fromFile("./config/SafetyLimitConfig.yaml");
        registry = std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>>();
        const std::string joint_topic_name = fmt::format("{}/{}", robot_name, "joint_command");
        zlc::registerSubscriberHandler(joint_topic_name, &ControlModeFactory::writeJointCommand,
                                       this);
        const std::string cartesian_topic_name =
            fmt::format("{}/{}", robot_name, "cartesian_pose_command");
        zlc::registerSubscriberHandler(cartesian_topic_name,
                                       &ControlModeFactory::writeCartesianCommand, this);
    }

    void registerControlModes(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                              AtomicDoubleBuffer<franka::RobotState>& state_buffer)
    {
        registry["Idle"] = std::make_unique<IdleControlMode>();
        registry["HumanControl"] = std::make_unique<HumanControlMode>();
        registry["HybridJointImpedance"] = std::make_unique<HybridJointImpedanceControl>(desired_joint_command_);
        registry["CartesianImpedance"] =
            std::make_unique<CartesianImpedanceController>(desired_cartesian_pose_);
        for (const auto& pair : registry)
        {
            zlc::info("[ControlModeFactory] Registered mode: {}", pair.first);
            pair.second->initController(robot, pinocchio_model, state_buffer, safety_config_);
        }
    }

    AbstractControlMode* getController(const std::string& mode_name)
    {
        auto it = registry.find(mode_name);
        if (it != registry.end())
        {
            return it->second.get();
        }
        return nullptr;
    }

  private:
    std::unordered_map<std::string, std::unique_ptr<AbstractControlMode>> registry;
    SafetyLimitConfig safety_config_;
    AtomicDoubleBuffer<JointPosition> desired_joint_command_{JointPosition::Zero()};
    AtomicDoubleBuffer<transform::Pose> desired_cartesian_pose_{transform::Pose::Identity()};

    void writeJointCommand(const JointCommand& cmd)
    {
        desired_joint_command_.write(JointPosition::Map(cmd.pos.data()));
    }

    void writeCartesianCommand(const CartesianPoseCommand& cmd)
    {
        const Eigen::Vector3d translation = Eigen::Vector3d::Map(cmd.pos.data());
        // Command quaternion convention is [x, y, z, w].
        const Eigen::Quaterniond quaternion(cmd.rot[3], cmd.rot[0], cmd.rot[1], cmd.rot[2]);
        desired_cartesian_pose_.write(transform::Pose(quaternion, translation));
    }
};
