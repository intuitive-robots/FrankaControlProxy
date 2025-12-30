// #include "control_mode/cartesian_pose_mode.hpp"
// #include "protocol/mode_id.hpp"
// #include "protocol/codec.hpp"
// #include <franka/exception.h>

// #include <unistd.h>
// //Note: Pose is represented as a 4x4 matrix in column-major format.
// CartesianPoseMode::CartesianPoseMode():
//     desired_pose_(franka::CartesianPose{
//         {1.0, 0.0, 0.0, 0.0,
//          0.0, 1.0, 0.0, 0.0,
//          0.0, 0.0, 1.0, 0.0,
//          0.306, 0.0, 0.485, 1.0}
//     })
// {};
// CartesianPoseMode::~CartesianPoseMode() = default;

// void CartesianPoseMode::controlLoop() {
//     zlc::info("[CartesianPoseMode] Started.");
//     is_running_ = true;

//     // Initialize desired Cartesian pose to identity (no movement)`
//     try
//     {
//         desired_pose_.write(franka::CartesianPose{
//         {1.0, 0.0, 0.0, 0.0,   // col 1 (rotationx 0)
//          0.0, 1.0, 0.0, 0.0,   // col 2 (rotationy 0)
//          0.0, 0.0, 1.0, 0.0,   // col 3 (rotationz 0)
//          0.306, 0.0, 0.485,1.0}   // col 4 (translation 1)
//     });
//     }
//     catch(const std::exception& ex)
//     {
//         zlc::error("[CartesianPoseMode] desired_pose set wrong: {}", ex.what());
//     }
    
//     zlc::info("[CartesianPoseMode] desired_pose set");
//     if (!robot_ || !model_) {
//         zlc::error("[CartesianPoseMode] Robot or model not set.");
//         return;
//     }

//     robot_->automaticErrorRecovery();

//     std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> motion_generator_callback =
//         [this](const franka::RobotState& state, franka::Duration) -> franka::CartesianPose {
//             // if (!is_running_) {
//             //     throw franka::ControlException("CartesianPoseMode stopped.");
//             // }
//             updateRobotState(state);
//             auto desired = desired_pose_.read();
//             if (!is_running_) {
//                 return franka::MotionFinished(desired);
//             }

//             return desired;
//         };
//     bool is_robot_operational = true;
//     while (is_running_ && is_robot_operational) {
//         try {
//             robot_->control(motion_generator_callback);
//     } catch (const std::exception &ex) {
//         zlc::error("[CartesianPoseMode] Robot is unable to be controlled: {}", ex.what());
//         is_robot_operational = false;
//     }
//     for (int i = 0; i < 3; i++) {
//         zlc::warn("[CartesianPoseMode] Waiting {} seconds before recovery attempt...", 3);

//         // Wait
//         usleep(100 * 3);
//         // Attempt recovery
//         try {
//             robot_->automaticErrorRecovery();
//             zlc::info("[CartesianPoseMode] Robot operation recovered.");
//             is_robot_operational = true;
//             break;
//             } catch (const std::exception &ex) {
//                 zlc::error("[CartesianPoseMode] Recovery failed: {}", ex.what());
//             }
//         }
    
//     }
// }

// protocol::ControlModeID CartesianPoseMode::getControlModeID() const {
//     return protocol::ControlModeID::CARTESIAN_POSE;
// }

// void CartesianPoseMode::writeCommand(const protocol::ByteView& data) {
//     franka::CartesianPose pose = protocol::decode<franka::CartesianPose>(data);
//     desired_pose_.write(pose);
// }

// void CartesianPoseMode::writeZeroCommand() {
//     franka::CartesianPose current_pose = current_state_->read().O_T_EE;
//     desired_pose_.write(current_pose);
// }
