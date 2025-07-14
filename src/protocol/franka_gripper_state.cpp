#include "protocol/franka_gripper_state.hpp"
#include "protocol/byte_order.hpp"
#include "protocol/codec.hpp"
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <franka/gripper_state.h>
namespace protocol {

std::vector<uint8_t> FrankaGripperState::gripper_encode() const {
        std::vector<uint8_t> buffer(kSize);
        uint8_t* ptr = buffer.data();

        encode_u32(ptr, timestamp_ms);
        encode_f64(ptr, width);
        encode_f64(ptr, max_width);
        encode_bool(ptr, grasped);
        encode_u16(ptr, temperature);

        return buffer;
    }

FrankaGripperState FrankaGripperState::gripper_decode(const uint8_t* buffer, size_t size) {
    if (size != kSize) {
        throw std::runtime_error("FrankaGripperState::gripper_decode() size mismatch");
    }

    FrankaGripperState state;
    const uint8_t* ptr = buffer;

    state.timestamp_ms = decode_u32(ptr);
    state.width = decode_f64(ptr);
    state.max_width = decode_f64(ptr);
    state.grasped = decode_bool(ptr);
    state.temperature = decode_u16(ptr);

    return state;
    }

    FrankaGripperState FrankaGripperState::fromGripperState(const franka::GripperState& gripper_state) {
    FrankaGripperState state;
    state.timestamp_ms = static_cast<uint32_t>(gripper_state.time.toMSec());
    state.width = gripper_state.width;
    state.max_width = gripper_state.max_width;
    state.grasped = gripper_state.is_grasped;
    state.temperature = static_cast<uint16_t>(gripper_state.temperature);

    return state;
}

franka::GripperState FrankaGripperState::toGripperState() const {
    franka::GripperState gripper_state;
    gripper_state.time = franka::Duration(timestamp_ms * 1000000);//need to be test
    gripper_state.width = width;
    gripper_state.max_width = max_width;
    gripper_state.is_grasped = grasped;
    gripper_state.temperature = static_cast<double>(temperature);

    return gripper_state;
}
}