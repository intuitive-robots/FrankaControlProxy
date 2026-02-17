#pragma once

#include <franka/command_types.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <zerolancom/zerolancom.hpp>

#include "mujoco_sim/mujoco_panda_env.hpp"
#include "mujoco_sim/mujoco_viewer.hpp"
#include "utils/robot_model.hpp"

class MujocoModel
{
  public:
    explicit MujocoModel(MujocoPandaEnv* env);

    mjModel* getModel() const
    {
        return env_->getModel();
    }

    mjData* getData() const
    {
        return env_->getData();
    }

  private:
    MujocoPandaEnv* env_;
};

class MujocoRobot
{
  public:
    explicit MujocoRobot(const std::string& franka_address);

    MujocoRobot(MujocoRobot&& other) noexcept;

    MujocoRobot& operator=(MujocoRobot&& other) noexcept;

    ~MujocoRobot() noexcept;

    void control(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                     control_callback,
                 bool limit_rate = true, double cutoff_frequency = franka::kMaxCutoffFrequency);

    void control(std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
                     motion_generator_callback,
                 bool limit_rate = true, double cutoff_frequency = franka::kMaxCutoffFrequency);

    void control(std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
                     motion_generator_callback,
                 bool limit_rate = true, double cutoff_frequency = franka::kMaxCutoffFrequency);

    void control(std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
                     motion_generator_callback,
                 bool limit_rate = true, double cutoff_frequency = franka::kMaxCutoffFrequency);

    void control(std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>
                     motion_generator_callback,
                 bool limit_rate = true, double cutoff_frequency = franka::kMaxCutoffFrequency);

    void read(std::function<bool(const franka::RobotState&)> read_callback);

    franka::RobotState readOnce();

    void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                              const std::array<double, 7>& upper_torque_thresholds_acceleration,
                              const std::array<double, 7>& lower_torque_thresholds_nominal,
                              const std::array<double, 7>& upper_torque_thresholds_nominal,
                              const std::array<double, 6>& lower_force_thresholds_acceleration,
                              const std::array<double, 6>& upper_force_thresholds_acceleration,
                              const std::array<double, 6>& lower_force_thresholds_nominal,
                              const std::array<double, 6>& upper_force_thresholds_nominal);

    void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                              const std::array<double, 7>& upper_torque_thresholds,
                              const std::array<double, 6>& lower_force_thresholds,
                              const std::array<double, 6>& upper_force_thresholds);

    void automaticErrorRecovery();

    void stop();

    franka::Model loadModel();

    MujocoRobot(const MujocoRobot&) = delete;
    MujocoRobot& operator=(const MujocoRobot&) = delete;

  private:
    // Control conversion helpers
    franka::Torques jointPositionToTorque(const franka::JointPositions& desired_positions,
                                          const franka::RobotState& state);
    franka::Torques jointVelocityToTorque(const franka::JointVelocities& desired_velocities,
                                          const franka::RobotState& state);
    std::array<double, 7> cartesianPoseToJointPosition(const franka::CartesianPose& desired_pose,
                                                       const franka::RobotState& state);
    std::array<double, 7> cartesianVelocityToJointVelocity(const franka::CartesianVelocities& desired_velocities,
                                                           const franka::RobotState& state);

    // Default PD gains (libfranka defaults)
    static constexpr std::array<double, 7> kDefaultStiffness = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
    static constexpr std::array<double, 7> kDefaultDamping = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};

    // IK parameters
    static constexpr double kIKErrorThreshold = 1e-4;
    static constexpr int kIKMaxIterations = 100;
    static constexpr double kIKStepSize = 0.5;

    bool running_{false};

    std::unique_ptr<MujocoPandaEnv> env_;
    std::unique_ptr<MujocoViewer> viewer_;
    std::unique_ptr<PandaPinocchioModel> model_;

    mutable std::mutex state_mutex_;
    std::mutex control_mutex_;

    franka::RobotState current_state_{};
};
