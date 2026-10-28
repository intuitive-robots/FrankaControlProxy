#pragma once
#include <control_mode/abstract_control_mode.hpp>
#include <control_mode/cartesian_impedance.hpp>
#include <control_mode/gravity_comp_control.hpp>
#include <control_mode/human_control.hpp>
#include <control_mode/hybrid_joint_impedance_control.hpp>
#include <control_mode/hybrid_joint_ext_torque_impedance_control.hpp>
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
        const std::string joint_torque_topic_name =
            fmt::format("{}/{}", robot_name, "joint_torque_command");
        zlc::registerSubscriberHandler(joint_torque_topic_name,
                                       &ControlModeFactory::writeJointTorqueCommand, this);
        const std::string cartesian_topic_name =
            fmt::format("{}/{}", robot_name, "cartesian_pose_command");
        zlc::registerSubscriberHandler(cartesian_topic_name,
                                       &ControlModeFactory::writeCartesianCommand, this);
        const std::string ee_wrench_topic_name =
            fmt::format("{}/{}", robot_name, "ee_wrench_command");
        zlc::registerSubscriberHandler(ee_wrench_topic_name,
                                       &ControlModeFactory::writeEEWrenchCommand, this);
    }

    void registerControlModes(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                              AtomicDoubleBuffer<franka::RobotState>& state_buffer)
    {
        registry["Idle"] = std::make_unique<IdleControlMode>();
        registry["HumanControl"] = std::make_unique<HumanControlMode>();
        registry["HybridJointImpedance"] =
            std::make_unique<HybridJointImpedanceControl>(desired_joint_command_, desired_ee_wrench_);
        // same controller with soft Cartesian gains for contact-rich teleoperation (insertion)
        registry["HybridJointImpedanceCompliant"] = std::make_unique<HybridJointImpedanceControl>(
            desired_joint_command_, desired_ee_wrench_,
            "config/controller/hybrid_joint_impedance_compliant_controller.yaml");
        registry["HybridJointExtTorqueImpedance"] = std::make_unique<HybridJointExtTorqueImpedanceControl>(desired_joint_command_, desired_joint_ext_torque_command_);
        registry["CartesianImpedance"] =
            std::make_unique<CartesianImpedanceController>(desired_cartesian_target_);
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
    AtomicDoubleBuffer<JointTarget> desired_joint_command_{JointTarget{}};
    AtomicDoubleBuffer<JointTorque> desired_joint_ext_torque_command_{JointTorque::Zero()};
    AtomicDoubleBuffer<CartesianTarget> desired_cartesian_target_{CartesianTarget{}};
    AtomicDoubleBuffer<EEWrenchTarget> desired_ee_wrench_{EEWrenchTarget{}};

    void writeJointCommand(const JointCommand& cmd)
    {
        JointTarget target;
        target.pos = JointPosition::Map(cmd.pos.data());
        // vel is optional in the message; clients that only send pos get a zero velocity reference
        target.vel = JointVelocity::Map(cmd.vel.data());
        target.stamp = std::chrono::steady_clock::now();
        desired_joint_command_.write(target);
    }

    void writeJointTorqueCommand(const JointTorqueCommand& cmd)
    {
        desired_joint_ext_torque_command_.write(JointTorque::Map(cmd.tau.data()));
    }

    void writeEEWrenchCommand(const EEWrenchCommand& cmd)
    {
        EEWrenchTarget target;
        target.wrench = transform::Vector6d::Map(cmd.wrench.data());
        target.stamp = std::chrono::steady_clock::now();
        desired_ee_wrench_.write(target);
    }

    void writeCartesianCommand(const CartesianPoseCommand& cmd)
    {
        const Eigen::Vector3d translation = Eigen::Vector3d::Map(cmd.pos.data());
        // Command quaternion convention is [x, y, z, w].
        const Eigen::Quaterniond quaternion(cmd.rot[3], cmd.rot[0], cmd.rot[1], cmd.rot[2]);
        CartesianTarget target;
        target.pose = transform::Pose(quaternion, translation);
        // pos_vel is the linear, rot_vel[0:3] the angular velocity, both in the base frame
        target.twist << Eigen::Vector3d::Map(cmd.pos_vel.data()), Eigen::Vector3d::Map(cmd.rot_vel.data());
        target.stamp = std::chrono::steady_clock::now();
        desired_cartesian_target_.write(target);
    }
};
