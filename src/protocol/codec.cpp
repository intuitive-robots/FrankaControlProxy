#include "protocol/codec.hpp"
#include "protocol/message_header.hpp" 
#include "protocol/msg_id.hpp"
#include "protocol/franka_arm_state.hpp"
#include "protocol/franka_gripper_state.hpp"
#include "protocol/mode_id.hpp"
#include "protocol/request_result.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <limits>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
namespace protocol {


// TODO: using header static consturctor for encode all the message?

// header + payload (12-byte header)
//payload only read
std::vector<uint8_t> encodeMessage(const MsgHeader& header, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> result(MsgHeader::SIZE + payload.size());
    header.encode(result.data());  // write header
    std::memcpy(result.data() + MsgHeader::SIZE, payload.data(), payload.size());
    return result;
}

// ------------------------------------------------------------
// Generic payload-level codec implementations
// ------------------------------------------------------------
//string:RequestResult payload
std::vector<uint8_t> encode(const std::string& v) {
    if (v.size() > static_cast<size_t>(std::numeric_limits<uint16_t>::max())) {
        throw std::runtime_error("string too long to encode");
    }
    const uint16_t len = static_cast<uint16_t>(v.size());
    std::vector<uint8_t> out(2 + len);
    uint8_t* wptr = out.data();
    encode_u16(wptr, len);
    if (len) {
        std::memcpy(wptr, v.data(), len);
    }
    return out;
}
//uint8_t:ModeID payload
std::vector<uint8_t> encode(uint8_t v) {
    return std::vector<uint8_t>{v};
}
//uint16_t:Pubport payload
std::vector<uint8_t> encode(uint16_t v) {
    std::vector<uint8_t> out(2);
    uint8_t* wptr = out.data();
    encode_u16(wptr, v);
    return out;
}

// franka::RobotState: FrankaArmState payload
std::vector<uint8_t> encode(const franka::RobotState& rs) {
    // 4 + 16*8 + 7*8 = 188 bytes
    std::vector<uint8_t> out(4 + 16 * sizeof(double) + 7 * sizeof(double));
    uint8_t* wptr = out.data();
    encode_u32(wptr, static_cast<uint32_t>(rs.time.toMSec()));
    encode_array_f64(wptr, rs.O_T_EE);
    encode_array_f64(wptr, rs.q);
    return out;
}



//string: FrankaArmControl payload(Mode_ID+URL)
template <>
std::string decode<std::string>(const std::vector<uint8_t>& payload) {
    if (payload.size() < 2) {
        throw std::runtime_error("decode<string>: payload too small");
    }
    const uint8_t* rptr = payload.data();
    const uint16_t len = decode_u16(rptr);
    const size_t expected = static_cast<size_t>(2 + len);
    if (payload.size() != expected) {
        throw std::runtime_error("decode<string>: length mismatch");
    }
    return std::string(reinterpret_cast<const char*>(rptr), len);
}
template <>
franka::JointPositions decode<franka::JointPositions>(const std::vector<uint8_t>& payload) {
    constexpr size_t kDoF = 7;
    constexpr size_t kSize = kDoF * sizeof(double);
    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::JointPositions>: payload size mismatch");
    }
    const uint8_t* rptr = payload.data();
    std::array<double, kDoF> q{};
    decode_array_f64(rptr, q);
    franka::JointPositions jp{};
    jp.q = q;
    return jp;
}
template <>
franka::JointVelocities decode<franka::JointVelocities>(const std::vector<uint8_t>& payload) {
    constexpr size_t kDoF = 7;
    constexpr size_t kSize = kDoF * sizeof(double);
    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::JointVelocities>: payload size mismatch");
    }
    const uint8_t* rptr = payload.data();
    std::array<double, kDoF> dq{};
    decode_array_f64(rptr, dq);
    franka::JointVelocities jv{};
    jv.dq = dq;
    return jv;
}
template <>
franka::CartesianPose decode<franka::CartesianPose>(const std::vector<uint8_t>& payload) {
    constexpr size_t kSize = 16 * sizeof(double);
    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::CartesianPose>: payload size mismatch");
    }
    const uint8_t* rptr = payload.data();
    std::array<double, 16> pose{};
    decode_array_f64(rptr, pose);
    return franka::CartesianPose{pose};
}

template <>
franka::CartesianVelocities decode<franka::CartesianVelocities>(const std::vector<uint8_t>& payload) {
    constexpr size_t kSize = 6 * sizeof(double);
    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::CartesianVelocities>: payload size mismatch");
    }
    const uint8_t* rptr = payload.data();
    std::array<double, 6> vel{};
    decode_array_f64(rptr, vel);
    return franka::CartesianVelocities{vel};
}






}  // namespace protocol
