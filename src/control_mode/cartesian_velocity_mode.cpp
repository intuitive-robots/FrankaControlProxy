#include "cartesian_velocity_mode.hpp"
#include <franka/exception.h>
#include <franka/control_types.h>
#include <iostream>
#include <unistd.h>

CartesianVelocityMode::CartesianVelocityMode():
    desired_velocities_(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
CartesianVelocityMode::~CartesianVelocityMode() = default;

void CartesianVelocityMode::controlLoop() {
    std::cout << "[CartesianVelocityMode] Started.\n";
    is_running_ = true;

    desired_velocities_.write(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        std::cerr << "[CartesianVelocityMode] Robot or model not set.\n";
        return;
    }

    robot_->automaticErrorRecovery();

    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

    std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)> motion_generator_callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianVelocities {
            updateRobotState(state);
            auto desired = desired_velocities_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired);
            }
            return desired;
        };
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [this, k_gains, d_gains](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.
      std::array<double, 7> coriolis = model_->coriolis(state);

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
    bool is_robot_operational = true;
    while (is_running_ && is_robot_operational) {
        try {
            // robot_->control(impedance_control_callback, motion_generator_callback, true, 1);
            robot_->control(motion_generator_callback, franka::ControllerMode::kCartesianImpedance, true, 1);
        } catch (const std::exception &ex) {
            std::cout << "Robot is unable to be controlled: " << ex.what() << std::endl;
            is_robot_operational = false;
        }
        for (int i = 0; i < 3; i++) {
            std::cout << "[CartesianVelocityMode] Waiting " << 3
                        << " seconds before recovery attempt...\n";

            // Wait
            usleep(1000 * 3);

            // Attempt recovery
            try {
                robot_->automaticErrorRecovery();
                std::cout << "[CartesianVelocityMode] Robot operation recovered.\n";
                is_robot_operational = true;
                break;
            } catch (const std::exception &ex) {
                std::cout << "[CartesianVelocityMode] Recovery failed: " << ex.what() << std::endl;
            }
        }
    }
}


const std::string& CartesianVelocityMode::getModeName() {
    return ModeID::CARTESIAN_VELOCITY;
}

void CartesianVelocityMode::writeCommand(const std::vector<uint8_t>& data) {
    // const uint8_t* data = reinterpret_cast<const uint8_t*>(data);
    const protocol::MsgHeader req_header = protocol::MsgHeader::decode(data.data());//get header

    std::vector<uint8_t> payload(data.begin() + protocol::MsgHeader::SIZE, data.end());//get payload
    protocol::MsgHeader header = protocol::MsgHeader::decode(data.data());
    franka::CartesianVelocities velocities = protocol::decode<franka::CartesianVelocities>(payload);
    std::cout << "[CartesianVelocityMode] Received velocities command: [";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << velocities.O_dP_EE[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    desired_velocities_.write(velocities);
}
