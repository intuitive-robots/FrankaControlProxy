#include "control_mode/joint_position_mode.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/codec.hpp"
#include <franka/exception.h>
#include <franka/control_types.h>

#include <unistd.h>

JointPositionMode::JointPositionMode():
    desired_positions_(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
JointPositionMode::~JointPositionMode() = default;

void JointPositionMode::controlLoop() {
    zlc::info("[JointPositionMode] Started.");
    is_running_ = true;
    // Example stiffness values for J1..J7
    std::array<double, 7> joint_stiffness = {
        600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0
    };

    // Set the impedance (joint stiffness)
    robot_->setJointImpedance(joint_stiffness);
    //wrong initialzation
    // desired_positions_.write(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_ || !current_state_){
        zlc::error("[JointPositionMode] Robot or model not set.");
        is_running_ = false;
        return;
    }
    writeZeroCommand();
    robot_->automaticErrorRecovery();

    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::JointPositions {
            // if (!is_running_) {
            //     throw franka::ControlException("JointPositionMode stopped.");
            // }
            updateRobotState(state);
            auto desired = desired_positions_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired);
            }
            // for (size_t i = 0; i < 7; i++) {
            //     zlc::info("[JointPositionMode] Desired position[{}]: {:.4f}, Current position[{}]: {:.4f}",
            //              i, desired.q[i], i, state.q[i]);

            // for (size_t i = 0; i < 7; i++) {
            //     if (std::abs(desired.q[i] - state.q[i]) <0.01)
            //         {
            //             desired.q[i] = state.q[i]; // hold position if close enough
            //         }
            //     }
            // }
            
            return desired;
        };
    bool is_robot_operational = true;
    while (is_running_ && is_robot_operational) {
        try {
            robot_->control(callback,franka::ControllerMode::kJointImpedance,true,1);
            // is_robot_operational = false; // exit after control returns
            //debug fake control loop
            // franka::RobotState fake_state = current_state_->read();
            // franka::Duration fake_duration{0};
            // auto ret = callback(fake_state, fake_duration);
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } catch (const std::exception &ex) {
            zlc::error("[JointPositionMode] Robot is unable to be controlled: {}", ex.what());
            is_robot_operational = false;
        }
        if (!is_robot_operational) {
            for (int i = 0; i < 3; i++) {
                zlc::warn("[JointPositionMode] Waiting {} seconds before recovery attempt...", 3);
                usleep(1000 * 3);
                try {
                    robot_->automaticErrorRecovery();
                    zlc::info("[JointPositionMode] Robot operation recovered.");
                    is_robot_operational = true;
                    break;
                } catch (const franka::Exception& recovery_error) {
                    zlc::error("[JointPositionMode] Recovery failed: {}", recovery_error.what());
                }
            }
        }
    }
}
//     try {
//         robot_->control(callback);
//     } catch (const franka::ControlException& e) {
//         zlc::error("[JointPositionMode] Exception: {}", e.what());
//         if (std::string(e.what()).find("reflex") != std::string::npos) {
//             zlc::warn("Reflex detected, attempting automatic recovery...");
//             try {
//                 robot_->automaticErrorRecovery();
//             } catch (const franka::Exception& recovery_error) {
//                 zlc::error("Recovery failed: {}", recovery_error.what());
//             }
//         }
//         zlc::info("[JointPositionMode] Exited.");
//     }
// }


protocol::ControlModeID JointPositionMode::getControlModeID() const {
    return protocol::ControlModeID::JOINT_POSITION;
}

void JointPositionMode::writeCommand(const protocol::ByteView& data) {
    franka::JointPositions positions = protocol::decode<franka::JointPositions>(data);
    desired_positions_.write(positions);
}

void JointPositionMode::writeZeroCommand() {
    franka::JointPositions current_positions = current_state_->read().q;
    desired_positions_.write(current_positions);
}
