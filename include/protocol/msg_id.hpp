// #pragma once
// #include <cstdint>

// namespace protocol {
//     enum class MsgID : uint8_t {
//         // ===== Client → Server : Service Request =====
//         GET_FRANKA_ARM_STATE                  = 0,    // Request a single state
//         GET_FRANKA_ARM_CONTROL_MODE           = 1,    // Ask for active control mode
//         SET_FRANKA_ARM_CONTROL_MODE           = 2,    // Switch to desired mode
//         GET_FRANKA_ARM_STATE_PUB_PORT         = 3,    // Query PUB port number
//         MOVE_FRANKA_ARM_TO_JOINT_POSITION     = 4, // Move robot to a specific joint position
//         MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION = 5, // Move robot to a specific cartesian position
//         GET_FRANKA_GRIPPER_STATE              = 6,    // Request a single state of gripper
//         MOVE_FRANKA_GRIPPER                   = 7,    // Move gripper
        
//         // ===== Server → Client : Topic Publish =====
//         FRANKA_ARM_STATE_PUB        = 100,  // Publish current robot arm state
//         FRANKA_GRIPPER_STATE_PUB    = 101,  // Publish current robot gripper state

//         // ===== Client → Server : Command Topics =====
//         JOINT_POSITION_CMD         = 110,  // Command robot joints to reach target positions
//         JOINT_VELOCITY_CMD         = 111,  // Command robot joints with velocity targets
//         CARTESIAN_POSE_CMD         = 112,  // Command robot end-effector to Cartesian pose
//         CARTESIAN_VELOCITY_CMD     = 113,  // Command robot end-effector with Cartesian velocities
//         JOINT_TORQUE_CMD           = 114,  // Command robot joints torques

//         // ===== Server → Client : Request Result =====
//         SUCCESS                = 200,  // Operation completed successfully
//         FAIL                   = 201,  // Generic failure
//         INVALID_ARG            = 202,  // Invalid or out-of-range argument
//         BUSY                   = 203,  // Device busy / temporary reject
//         UNSUPPORTED            = 204,  // Command not supported in current mode
//         TIMEOUT                = 205,  // Operation timed out
//         COMM_ERROR             = 206,  // Communication or CRC error
//         INTERNAL_ERROR         = 207,  // Internal logic or hardware fault
//     };
// }  // namespace protocol
