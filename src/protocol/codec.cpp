#include "protocol/codec.hpp"
#include "protocol/msg_header.hpp" 
#include "protocol/msg_id.hpp"
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

// header + payload (12-byte header)
//payload only read
// std::vector<uint8_t> encodeMessage(const MsgHeader& header, const uint8_t* payload) {
//     std::vector<uint8_t> result(MsgHeader::SIZE + payload.size());
//     header.encode(result.data());  // write header
//     std::memcpy(result.data() + MsgHeader::SIZE, payload.data(), payload.size());
//     return result;
// }

// ------------------------------------------------------------
// Generic payload-level codec implementations
// ------------------------------------------------------------
//string:
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
    // Layout (bytes):
    // 0   : uint32  timestamp_ms
    // 4   : 16*f64  O_T_EE
    // 132 : 16*f64  O_T_EE_d
    // 260 : 7*f64   q
    // 316 : 7*f64   q_d
    // 372 : 7*f64   dq
    // 428 : 7*f64   dq_d
    // 484 : 7*f64   tau_ext_hat_filtered
    // 540 : 6*f64   O_F_ext_hat_K
    // 588 : 6*f64   K_F_ext_hat_K
    // Total = 636 bytes
    const size_t total_size = 4
        + 16 * sizeof(double)
        + 16 * sizeof(double)
        + 7 * sizeof(double) * 5
        + 6 * sizeof(double) * 2;
    std::vector<uint8_t> out(total_size);
    uint8_t* wptr = out.data();
    encode_u32(wptr, static_cast<uint32_t>(rs.time.toMSec()));
    encode_array_f64(wptr, rs.O_T_EE);
    encode_array_f64(wptr, rs.O_T_EE_d);
    encode_array_f64(wptr, rs.q);
    encode_array_f64(wptr, rs.q_d);
    encode_array_f64(wptr, rs.dq);
    encode_array_f64(wptr, rs.dq_d);
    encode_array_f64(wptr, rs.tau_ext_hat_filtered);
    encode_array_f64(wptr, rs.O_F_ext_hat_K);
    encode_array_f64(wptr, rs.K_F_ext_hat_K);
    return out;
}

std::vector<uint8_t> encode(const franka::GripperState& gs) {
    // Layout (bytes):
    // 0   : f64  width
    // 8   : f64  max_width
    // 16  : bool is_grasped
    // 17  : u16  temperature
    // Total = 19 bytes
    const size_t total_size = sizeof(double) * 2 + sizeof(bool) + sizeof(uint16_t);
    std::vector<uint8_t> out(total_size);
    uint8_t* wptr = out.data();
    encode_f64(wptr, gs.width);
    encode_f64(wptr, gs.max_width);
    encode_bool(wptr, gs.is_grasped);
    encode_u16(wptr, gs.temperature);
    return out;
}

// ============================================================================
// std::string decode
// ============================================================================
template <>
std::string decode<std::string>(ByteView payload) {
    constexpr size_t kMaxLen = 1024;

    if (!payload.data || payload.size == 0)
        return std::string();

    size_t real_len = 0;
    while (real_len < payload.size && real_len < kMaxLen && payload.data[real_len] != 0) {
        ++real_len;
    }

    return std::string(reinterpret_cast<const char*>(payload.data), real_len);
}


// ============================================================================
// uint16_t decode
// ============================================================================
template <>
uint16_t decode<uint16_t>(ByteView payload) {
    if (payload.size < sizeof(uint16_t)) {
        throw std::runtime_error("decode<uint16_t>: payload too small");
    }

    const uint8_t* rptr = payload.data;
    return decode_u16(rptr);
}


// ============================================================================
// franka control objects
// ============================================================================

template <>
franka::JointPositions decode<franka::JointPositions>(ByteView payload) {
    constexpr size_t kDoF = 7;
    constexpr size_t kNeed = kDoF * sizeof(double);

    if (payload.size < kNeed)
        throw std::runtime_error("decode<JointPositions>: payload too small");

    const uint8_t* rptr = payload.data;
    std::array<double, kDoF> q{};
    decode_array_f64(rptr, q);
    return franka::JointPositions(q);
}


template <>
franka::JointVelocities decode<franka::JointVelocities>(ByteView payload) {
    constexpr size_t kDoF = 7;
    constexpr size_t kNeed = kDoF * sizeof(double);

    if (payload.size < kNeed)
        throw std::runtime_error("decode<JointVelocities>: payload too small");

    const uint8_t* rptr = payload.data;
    std::array<double, kDoF> dq{};
    decode_array_f64(rptr, dq);
    return franka::JointVelocities(dq);
}


template <>
franka::CartesianPose decode<franka::CartesianPose>(ByteView payload) {
    constexpr size_t kNeed = 16 * sizeof(double);

    if (payload.size < kNeed)
        throw std::runtime_error("decode<CartesianPose>: payload too small");

    const uint8_t* rptr = payload.data;
    std::array<double, 16> pose{};
    decode_array_f64(rptr, pose);
    return franka::CartesianPose{pose};
}


template <>
franka::CartesianVelocities decode<franka::CartesianVelocities>(ByteView payload) {
    constexpr size_t kNeed = 6 * sizeof(double);

    if (payload.size < kNeed)
        throw std::runtime_error("decode<CartesianVelocities>: payload too small");

    const uint8_t* rptr = payload.data;
    std::array<double, 6> vel{};
    decode_array_f64(rptr, vel);
    return franka::CartesianVelocities{vel};
}


template <>
franka::Torques decode<franka::Torques>(ByteView payload) {
    constexpr size_t kDoF = 7;
    constexpr size_t kNeed = kDoF * sizeof(double);

    if (payload.size < kNeed)
        throw std::runtime_error("decode<Torques>: payload too small");

    const uint8_t* rptr = payload.data;
    std::array<double, kDoF> tau{};
    decode_array_f64(rptr, tau);
    return franka::Torques{tau};
}


// ============================================================================
// protocol::FrankaArmControlMode
// ============================================================================

template <>
protocol::FrankaArmControlMode decode<protocol::FrankaArmControlMode>(ByteView payload) {
    if (payload.size == 0)
        throw std::runtime_error("decode<FrankaArmControlMode>: payload empty");

    protocol::FrankaArmControlMode output;
    output.id = static_cast<protocol::ModeID>(payload.data[0]);

    if (payload.size > 1) {
        output.url = std::string(
            reinterpret_cast<const char*>(payload.data + 1),
            payload.size - 1
        );
    }

    return output;
}

template <>
protocol::GraspCommand decode<protocol::GraspCommand>(ByteView payload) {
    if (payload.size != 3 * sizeof(double))
        throw std::runtime_error("decode<GraspCommand>: payload too small");
    const double* rptr = reinterpret_cast<const double*>(payload.data);
    protocol::GraspCommand cmd;
    cmd.width = protocol::from_big_endian_f64(*rptr);
    rptr++;
    cmd.speed = protocol::from_big_endian_f64(*rptr);
    rptr++;
    cmd.force = protocol::from_big_endian_f64(*rptr);
    return cmd;
}  // namespace protocol

}