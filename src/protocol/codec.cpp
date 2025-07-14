#include "protocol/codec.hpp"
#include "protocol/message_header.hpp" 
#include "protocol/msg_id.hpp"
#include "protocol/franka_arm_state.hpp"
#include "protocol/franka_gripper_state.hpp"
#include "protocol/mode_id.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
namespace protocol {

// header + payload
std::vector<uint8_t> encodeMessage(const MessageHeader& header, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> result(4 + payload.size());
    header.encode(result.data());  // get head
    std::memcpy(result.data() + 4, payload.data(), payload.size());
    return result;
}

// Arm:GET_STATE_RESP/PUB_STATE
std::vector<uint8_t> encodeStateMessage(const protocol::FrankaArmState& state) {
    auto payload = state.encode();  // 636B
    MessageHeader header{
        static_cast<uint8_t>(MsgID::GET_STATE_RESP), 
        static_cast<uint16_t>(payload.size())
    };
    return encodeMessage(header, payload);
}

//Gripper:GET_STATE_RESP/PUB_STATE
std::vector<uint8_t> encodeGripperMessage(const FrankaGripperState& gripper_state) {
    auto payload = gripper_state.gripper_encode();  // 23B
    MessageHeader header{
        static_cast<uint8_t>(MsgID::GET_STATE_RESP), 
        static_cast<uint16_t>(payload.size())
    };
    return encodeMessage(header, payload);
}


// QUERY_STATE_RESP
std::vector<uint8_t> encodeModeMessage(uint8_t mode_code) {
    std::vector<uint8_t> payload{mode_code}; 
    MessageHeader header{
        static_cast<uint8_t>(MsgID::GET_CONTROL_MODE_RESP),
        static_cast<uint16_t>(payload.size())
    }; // 1 Byte for mode code
    return encodeMessage(header, payload);
}

//START_CONTROL_RESP
// std::vector<uint8_t> encodeRespcontrolMessage() {

// }
std::vector<uint8_t> encodeStartControlResp(bool success, ModeID mode_id) {
    MessageHeader header;
    header.id = static_cast<uint16_t>(protocol::MsgID::SET_CONTROL_MODE_RESP);
    header.len = 2;
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>(success ? 0x00 : 0x01),
        static_cast<uint8_t>(mode_id)
    };
    return protocol::encodeMessage(header, payload);
}


// ERROR
std::vector<uint8_t> encodeErrorMessage(uint8_t error_code) {
    std::vector<uint8_t> payload{error_code};
    MessageHeader header{
        static_cast<uint8_t>(MsgID::ERROR),
        static_cast<uint16_t>(payload.size())
    };
    return encodeMessage(header, payload);
}

//Arm:SUB_STATE need to check
bool decodeStateMessage(const std::vector<uint8_t>& data, FrankaArmState& arm_state) {
    if (data.size() != FrankaArmState::kSize + MessageHeader::SIZE) {
        return false; // Size mismatch
    }
    const uint8_t* buffer = data.data() + MessageHeader::SIZE; // Skip header
    try {
        arm_state = FrankaArmState::decode(buffer, FrankaArmState::kSize);
        return true;
    } catch (const std::runtime_error& e) {
        std::cerr << "[FrankaProxy] Decode error: " << e.what() << std::endl;
        return false;
    }
}
// Gripper:SUB_STATE need to check
bool decodeGripperMessage(const std::vector<uint8_t>& data, FrankaGripperState& gripper_state) {
    if (data.size() != FrankaGripperState::kSize + MessageHeader::SIZE) {
        return false; // Size mismatch
    }
    const uint8_t* buffer = data.data() + MessageHeader::SIZE; // Skip header
    try {
        gripper_state = FrankaGripperState::gripper_decode(buffer, FrankaGripperState::kSize);
        return true;
    } catch (const std::runtime_error& e) {
        std::cerr << "[FrankaProxy] Decode error: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace protocol