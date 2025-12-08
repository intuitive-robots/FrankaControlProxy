#include "protocol/codec.hpp"
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
#include "utils/logger.hpp"
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
    wptr += sizeof(uint32_t);
    encode_array_f64(wptr, rs.O_T_EE);
    wptr += 16 * sizeof(double);
    encode_array_f64(wptr, rs.O_T_EE_d);
    wptr += 16 * sizeof(double);
    encode_array_f64(wptr, rs.q);
    wptr += 7 * sizeof(double);
    encode_array_f64(wptr, rs.q_d);
    wptr += 7 * sizeof(double);
    encode_array_f64(wptr, rs.dq);
    wptr += 7 * sizeof(double);
    encode_array_f64(wptr, rs.dq_d);
    wptr += 7 * sizeof(double);
    encode_array_f64(wptr, rs.tau_ext_hat_filtered);
    wptr += 7 * sizeof(double);
    encode_array_f64(wptr, rs.O_F_ext_hat_K);
    wptr += 6 * sizeof(double);
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
    wptr += sizeof(double);
    encode_f64(wptr, gs.max_width);
    wptr += sizeof(double);
    encode_bool(wptr, gs.is_grasped);
    wptr += sizeof(bool);
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
    std::cout<< "q command:";
    for (auto v:q){
        std::cout<<v<< " ";
    }
    std::cout<<std::endl;
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
franka::RobotState decode<franka::RobotState>(ByteView payload) {
    // Must match encode layout in this file
    const size_t total_size = 4
        + 16 * sizeof(double)
        + 16 * sizeof(double)
        + 7 * sizeof(double) * 5
        + 6 * sizeof(double) * 2;

    if (payload.size < total_size) {
        throw std::runtime_error("decode<RobotState>: payload too small");
    }

    const uint8_t* rptr = payload.data;
    franka::RobotState rs; // default-initialized

    // timestamp (ms)
    uint32_t ts_ms = decode_u32(rptr);
    rptr += sizeof(uint32_t);
    // Set time (franka::Duration accepts seconds in libfranka)
    rs.time = franka::Duration(static_cast<double>(ts_ms) / 1000.0);

    // helper temporaries to decode arrays then copy into C-arrays in RobotState
    std::array<double, 16> a16{};
    std::array<double, 7> a7{};
    std::array<double, 6> a6{};
    printf("addr1=%p\n",rptr);
    decode_array_f64(rptr, a16);
    rptr += 16 * sizeof(double);
    for (size_t i = 0; i < 16; ++i) rs.O_T_EE[i] = a16[i];
    printf("addr2=%p\n",rptr);
    decode_array_f64(rptr, a16);
    rptr += 16 * sizeof(double);
    for (size_t i = 0; i < 16; ++i) rs.O_T_EE_d[i] = a16[i];

    // q, q_d, dq, dq_d, tau_ext_hat_filtered  (7 doubles each)
    printf("addr3=%p\n",rptr);
    decode_array_f64(rptr, a7);
    rptr += 7 * sizeof(double);
    for (size_t i = 0; i < 7; ++i) rs.q[i] = a7[i];

    decode_array_f64(rptr, a7);
    rptr += 7 * sizeof(double);
    for (size_t i = 0; i < 7; ++i) rs.q_d[i] = a7[i];

    decode_array_f64(rptr, a7);
    rptr += 7 * sizeof(double);
    for (size_t i = 0; i < 7; ++i) rs.dq[i] = a7[i];

    decode_array_f64(rptr, a7);
    rptr += 7 * sizeof(double);
    for (size_t i = 0; i < 7; ++i) rs.dq_d[i] = a7[i];

    decode_array_f64(rptr, a7);
    rptr += 7 * sizeof(double);
    for (size_t i = 0; i < 7; ++i) rs.tau_ext_hat_filtered[i] = a7[i];

    // O_F_ext_hat_K and K_F_ext_hat_K (6 doubles each)
    decode_array_f64(rptr, a6);
    rptr += 6 * sizeof(double);
    for (size_t i = 0; i < 6; ++i) rs.O_F_ext_hat_K[i] = a6[i];

    decode_array_f64(rptr, a6);
    for (size_t i = 0; i < 6; ++i) rs.K_F_ext_hat_K[i] = a6[i];

    return rs;
}
// ...existing code...
template <>
protocol::GraspCommand decode<protocol::GraspCommand>(ByteView payload) {
    if (payload.size != 3 * sizeof(double))
        throw std::runtime_error("decode<GraspCommand>: payload too small");
    const double* rptr = reinterpret_cast<const double*>(payload.data);
    protocol::GraspCommand cmd;
    cmd.width = protocol::from_big_endian_f64(*rptr);
    rptr += sizeof(double);
    cmd.speed = protocol::from_big_endian_f64(*rptr);
    rptr += sizeof(double);
    cmd.force = protocol::from_big_endian_f64(*rptr);
    return cmd;
}  // namespace protocol

}