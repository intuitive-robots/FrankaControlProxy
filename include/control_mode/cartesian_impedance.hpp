#pragma once
#include <array>
#include <msgpack.hpp>

#include "control_mode/abstract_control_mode.hpp"
#include "protocol/control_command.hpp"
#include "utils/linear_trajInterpolator.hpp"

struct CartesianImpedanceConfig : public ControllerConfig
{
    Eigen::Matrix<double, 3, 3> Kp_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kp_r{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_p{Eigen::Matrix<double, 3, 3>::Zero()};
    Eigen::Matrix<double, 3, 3> Kd_r{Eigen::Matrix<double, 3, 3>::Zero()};
    bool ignore_gravity{true};
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

        // Compute Kd from Kp (critically damped)
        Kd_p = Kp_p.cwiseSqrt() * 2.0;
        Kd_r = Kp_r.cwiseSqrt() * 2.0;

        ignore_gravity = reader.getValue<bool>("ignore_gravity");
    }
};

class CartesianImpedanceController : public AbstractControlMode
{
  public:
    CartesianImpedanceController(AtomicDoubleBuffer<transform::Pose>& desired_cartesian_pose)
        : desired_cartesian_pose_(&desired_cartesian_pose)
    {
        controller_name = "CartesianImpedance";
    }
    ~CartesianImpedanceController() override = default;

  private:
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
    void startControl() override;
    CartesianImpedanceConfig config_;
    AtomicDoubleBuffer<transform::Pose>* desired_cartesian_pose_ = nullptr;
};
