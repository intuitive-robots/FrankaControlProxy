#pragma once
#include <array>
#include <chrono>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "protocol/control_command.hpp"
#include "utils/Pose.h"

// Desired pose and feed-forward twist (base frame: v [m/s], omega [rad/s]) of one command,
// stored together so the control loop never mixes pose and twist of different commands.
struct CartesianTarget
{
    transform::Pose pose{transform::Pose::Identity()};
    Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};
    std::chrono::steady_clock::time_point stamp{};
};

struct CartesianImpedanceConfig : public ControllerConfig
{
    Eigen::Matrix<double, 3, 3> Kp_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kp_r{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_r{Eigen::Matrix<double, 3, 3>::Zero()};
    double max_torque{5.0};
    // Scale of the commanded twist used as damping reference and for pose extrapolation (0 = off)
    double velocity_feedforward{1.0};
    // Shift the desired pose along the commanded twist between commands
    bool extrapolate_pose{true};
    // Feed-forward and extrapolation stop when the last command is older than this [s]
    double feedforward_timeout{0.1};
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


        // // Compute Kd from Kp (critically damped)
        // Kd_p = Kp_p.cwiseSqrt() * 2.0;
        // Kd_r = Kp_r.cwiseSqrt() * 2.0;
        max_torque = reader.getValue<double>("max_torque");
        velocity_feedforward = reader.getValue<double>("velocity_feedforward", 1.0);
        extrapolate_pose = reader.getValue<bool>("extrapolate_pose", true);
        feedforward_timeout = reader.getValue<double>("feedforward_timeout", 0.1);
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
                                franka::Duration /*duration*/) override;
    void startControl() override;
    CartesianImpedanceConfig config_;
    AtomicDoubleBuffer<CartesianTarget>* desired_cartesian_target_ = nullptr;
};
