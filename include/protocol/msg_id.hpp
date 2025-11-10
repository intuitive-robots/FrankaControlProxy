#pragma once
#include <cstdint>

namespace protocol {
    enum class MsgID : uint8_t {
    // ===== Client → Server : Service Request =====
    GET_FRANKA_ARM_STATE                  = 0,    // Request a single state
    GET_FRANKA_ARM_CONTROL_MODE           = 1,    // Ask for active control mode
    SET_FRANKA_ARM_CONTROL_MODE           = 2,    // Switch to desired mode
    GET_FRANKA_ARM_STATE_PUB_PORT         = 3,    // Query PUB port number
    MOVE_FRANKA_ARM_TO_JOINT_POSITION     = 4, // Move robot to a specific joint position
    MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION = 5, // Move robot to a specific cartesian position
    GET_FRANKA_GRIPPER_STATE              = 6,    // Request a single state of gripper
    MOVE_FRANKA_GRIPPER                   = 7,    // Move gripper

    // ===== Server → Client : Request Result =====
    SUCCESS                = 200,  // Operation completed successfully
    FAIL                   = 201,  // Generic failure
    INVALID_ARG            = 202,  // Invalid or out-of-range argument
    BUSY                   = 203,  // Device busy / temporary reject
    UNSUPPORTED            = 204,  // Command not supported in current mode
    TIMEOUT                = 205,  // Operation timed out
    COMM_ERROR             = 206,  // Communication or CRC error
    INTERNAL_ERROR         = 207,  // Internal logic or hardware fault

    // ===== Server → Client : Topic Publish =====
    FRANKA_ARM_STATE_PUB        = 100,  // Publish current robot arm state
    FRANKA_GRIPPER_STATE_PUB    = 101,  // Publish current robot gripper state

    // ===== Client → Server : Command Topics =====
    JOINT_POSITION_CMD         = 110,  // Command robot joints to reach target positions
    JOINT_VELOCITY_CMD         = 111,  // Command robot joints with velocity targets
    CARTESIAN_POSE_CMD         = 112,  // Command robot end-effector to Cartesian pose
    CARTESIAN_VELOCITY_CMD     = 113,  // Command robot end-effector with Cartesian velocities
    JOINT_TORQUE_CMD           = 114,  // Command robot joints torques
};



//10.30todo: change according to the current protocol
// enum class MsgID : uint8_t {
//     // Client → Server
//     GET_STATE_REQ      = 0x01,  // Request a single FrankaArmState
//     GET_CONTROL_MODE_REQ   = 0x02,  // Ask for active control mode
//     SET_CONTROL_MODE_REQ  = 0x03,  // Switch to desired mode
//     GET_SUB_PORT_REQ   = 0x04,  // Query PUB port number
//     GRIPPER_COMMAND_REQ = 0x05,  // Gripper command todo:add in client 

//     // Server → Client
//     GET_STATE_RESP     = 0x51,  //Respond to GET_STATE_REQ with FrankaArmState
//     GET_CONTROL_MODE_RESP   = 0x52,  //Respond to QUERY_STATE_REQ (1 byte: ControlMode)
//     SET_CONTROL_MODE_RESP = 0x53,  //Respond to START_CONTROL_REQ (1 byte: status,0 = OK)
//     GET_SUB_PORT_RESP  = 0x54,  // Respond to GET_SUB_PORT_REQ (2 bytes: port number)

//     // Server → Client (error)
//     ERROR              = 0xFF   // 1 byte error code
//     //error details
// };

}  // namespace protocol
