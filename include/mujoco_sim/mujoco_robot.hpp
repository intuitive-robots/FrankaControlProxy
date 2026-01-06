#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <franka/command_types.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <mujoco/mujoco.h>
#include <zerolancom/zerolancom.hpp>

#include "mujoco_sim/mujoco_panda_env.hpp"
#include "mujoco_sim/mujoco_viewer.hpp"

class MujocoModel {
public:
    MujocoModel() {
        zlc::info("Initialized MujocoModel for testing purposes.");
    }
    ~MujocoModel() = default;
    // Add other necessary fake methods as needed for testing
    std::array<double, 7> coriolis(const franka::RobotState& state) {
        return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

};

class MujocoRobot {
 public:

  explicit MujocoRobot(const std::string& franka_address);

  MujocoRobot(MujocoRobot&& other) noexcept;

  MujocoRobot& operator=(MujocoRobot&& other) noexcept;

  ~MujocoRobot() noexcept;


  void control(std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback,
               bool limit_rate = true,
               double cutoff_frequency = franka::kMaxCutoffFrequency);

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
  bool running_{false};

  std::unique_ptr<MujocoPandaEnv> env_;
  std::unique_ptr<MujocoViewer> viewer_;

  mutable std::mutex state_mutex_;
  std::mutex control_mutex_;

  franka::RobotState current_state_{};
};
