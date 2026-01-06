#include "control_mode/cartesian_velocity_mode.hpp"


#include <franka/exception.h>
#include <franka/control_types.h>

#include <unistd.h>

CartesianVelocityMode::CartesianVelocityMode():
    desired_velocities_(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}),
    config(CartesianVelocityConfig("config/controller/CartesianVelocityController.yaml"))
{
};


CartesianVelocityMode::~CartesianVelocityMode() = default;


void CartesianVelocityMode::initController() {
    zlc::registerSubscriberHandler(
        config.command_topic,
        &CartesianVelocityMode::writeCommand,
        this
    );
}

void CartesianVelocityMode::writeCommand(const CartesianVelocityCommand& cmd) {
    if (cmd.vel.size() != 6) {
        zlc::error("[CartesianVelocityMode] Received invalid command size: {}, expected 6.",
                    cmd.vel.size());
        return;
    }
    franka::CartesianVelocities velocities = franka::CartesianVelocities(cmd.vel);
    desired_velocities_.write(velocities);
}

void CartesianVelocityMode::controlLoop() {
    zlc::info("[CartesianVelocityMode] Started.");
    desired_velocities_.write(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)> motion_generator_callback =
    [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianVelocities {
        this->state_buffer_->write(state);
        auto desired = desired_velocities_.read();
        if (!is_running_) {
            return franka::MotionFinished(desired);
        }
        return desired;
    };
    
    // Stiffness
    std::array<double, 7> k_gains = config.k_gains;
    // Damping
    std::array<double, 7> d_gains = config.d_gains;
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [this, k_gains, d_gains](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.
      std::array<double, 7> coriolis =  model_->coriolis(state);
      

      // Compute torque command from joint impedance control law.
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
      }
      // Send torque command.
      return tau_d_calculated;
    };

    while (is_running_) {
        try {
            // robot_->control(impedance_control_callback, motion_generator_callback, true, 1);
            robot_->control(motion_generator_callback, franka::ControllerMode::kCartesianImpedance, config.limit_rate, config.cutoff_frequency);
        } catch (const std::exception &ex) {
            zlc::error("[CartesianVelocityMode] Robot is unable to be controlled: {}", ex.what());
            break;
        }
        bool recovered = tryRecovery();
        if (!recovered) {
            zlc::error("[CartesianVelocityMode] Unable to recover robot. Exiting control loop.");
            break;
        }
    }
}
