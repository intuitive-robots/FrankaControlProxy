#include "cartesian_velocity_mode.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"
#include <franka/exception.h>
#include <franka/control_types.h>
#include <iostream>

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
    try {
        robot_->control(impedance_control_callback, motion_generator_callback, 1);
    } catch (const franka::ControlException& e) {
        std::cerr << "[CartesianVelocityMode] Exception: " << e.what() << std::endl;
        if (std::string(e.what()).find("reflex") != std::string::npos) {
            std::cout << "Reflex detected, attempting automatic recovery...\n";
            try {
                robot_->automaticErrorRecovery();
            } catch (const franka::Exception& recovery_error) {
                std::cerr << "Recovery failed: " << recovery_error.what() << std::endl;
            }
        }
        std::cout << "[CartesianVelocityMode] Exited.\n";
    }
}


protocol::ModeID CartesianVelocityMode::getModeID() const {
    return protocol::ModeID::CARTESIAN_VELOCITY;
}

void CartesianVelocityMode::writeCommand(const std::vector<uint8_t>& data) {
    // const uint8_t* data = reinterpret_cast<const uint8_t*>(data);
    const protocol::MsgHeader req_header = protocol::MsgHeader::decode(data.data());//get header

    // Validate payload length
    // const size_t expect = static_cast<size_t>(MsgHeader::SIZE) + req_header.payload_length;
    // if (request.size() != expect) {
    //     response = RequestResult(RequestResultCode::INVALID_ARG, "Truncated payload").encodeMessage();
    //     return;
    // }
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
