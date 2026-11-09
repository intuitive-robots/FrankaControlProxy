#pragma once
#include <array>
#include <chrono>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "control_mode/cartesian_target.hpp"
#include "protocol/control_command.hpp"
#include "utils/Pose.h"

struct CartesianImpedanceConfig : public ControllerConfig
{
    Eigen::Matrix<double, 3, 3> Kp_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kp_r{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_r{Eigen::Matrix<double, 3, 3>::Zero()};
    double max_torque{5.0};
    // Twist feed-forward, pose extrapolation and the reference trajectory
    CartesianFeedforwardConfig feedforward;
    CartesianImpedanceConfig() = default;

    void fromFile(const std::string& controller_config_path) override
    {
        ConfigFileReader reader(controller_config_path);
        readBaseConfig(reader);

        // Read Kp_p and Kp_r diagonal values
        const std::array<double, 3> Kp_p_vals = reader.getArray<double, 3>("Kp_p");
        Kp_p = Eigen::Vector3d::Map(Kp_p_vals.data()).asDiagonal();
        const std::array<double, 3> Kp_r_vals = reader.getArray<double, 3>("Kp_r");
        Kp_r = Eigen::Vector3d::Map(Kp_r_vals.data()).asDiagonal();
        const std::array<double, 3> Kd_p_vals = reader.getArray<double, 3>("Kd_p");
        Kd_p = Eigen::Vector3d::Map(Kd_p_vals.data()).asDiagonal();
        const std::array<double, 3> Kd_r_vals = reader.getArray<double, 3>("Kd_r");
        Kd_r = Eigen::Vector3d::Map(Kd_r_vals.data()).asDiagonal();

        max_torque = reader.getValue<double>("max_torque");
        feedforward.read(reader);
    }
};

class CartesianImpedanceController : public AbstractControlMode
{
  public:
    CartesianImpedanceController(AtomicDoubleBuffer<CartesianTarget>& desired_cartesian_target)
        : desired_cartesian_target_(&desired_cartesian_target)
    {
        controller_name = "CartesianImpedance";
    }
    ~CartesianImpedanceController() override = default;

  private:
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
    void startControl() override;
    CartesianImpedanceConfig config_;
    CartesianReferenceGenerator reference_generator_;
    AtomicDoubleBuffer<CartesianTarget>* desired_cartesian_target_ = nullptr;
};
