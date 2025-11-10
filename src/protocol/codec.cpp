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
std::vector<uint8_t> encodeMessage(const MsgHeader& header, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> result(MsgHeader::SIZE + payload.size());
    header.encode(result.data());  // write header
    std::memcpy(result.data() + MsgHeader::SIZE, payload.data(), payload.size());
    return result;
}

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

std::vector<uint8_t> encode(const protocol::RequestResult& rr) {
    std::vector<uint8_t> out(1);
    out[0] = static_cast<uint8_t>(rr.code());
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

// string: FrankaArmControl payload(Mode_ID+URL)
void decode(const std::vector<uint8_t>& payload, std::string& output) {
    if (payload.size() < 2) {
        throw std::runtime_error("decode<string>: payload too small");
    }

    const uint8_t* rptr = payload.data();
    const uint16_t len = decode_u16(rptr);
    const size_t expected = static_cast<size_t>(2 + len);
    if (payload.size() != expected) {
        throw std::runtime_error("decode<string>: length mismatch");
    }

    output.assign(reinterpret_cast<const char*>(rptr), len);
}

void decode(const std::vector<uint8_t>& payload, uint8_t& output) {
    if (payload.size() != 1) {
        throw std::runtime_error("decode<uint8_t>: payload size mismatch");
    }
    output = payload[0];
}

void decode(const std::vector<uint8_t>& payload, uint16_t& output) {
    if (payload.size() != 2) {
        throw std::runtime_error("decode<uint16_t>: payload size mismatch");
    }
    const uint8_t* rptr = payload.data();
    output = decode_u16(rptr);
}

// ============================================================================
// libfranka control types
// ============================================================================

void decode(const std::vector<uint8_t>& payload, franka::JointPositions& output) {
    constexpr size_t kDoF = 7;
    constexpr size_t kSize = kDoF * sizeof(double);

    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::JointPositions>: payload size mismatch");
    }

    const uint8_t* rptr = payload.data();
    std::array<double, kDoF> q{};
    decode_array_f64(rptr, q);
    output.q = q;
}

void decode(const std::vector<uint8_t>& payload, franka::JointVelocities& output) {
    constexpr size_t kDoF = 7;
    constexpr size_t kSize = kDoF * sizeof(double);

    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::JointVelocities>: payload size mismatch");
    }

    const uint8_t* rptr = payload.data();
    std::array<double, kDoF> dq{};
    decode_array_f64(rptr, dq);
    output.dq = dq;
}

void decode(const std::vector<uint8_t>& payload, franka::CartesianPose& output) {
    constexpr size_t kSize = 16 * sizeof(double);

    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::CartesianPose>: payload size mismatch");
    }

    const uint8_t* rptr = payload.data();
    std::array<double, 16> pose{};
    decode_array_f64(rptr, pose);
    output = franka::CartesianPose{pose};
}

void decode(const std::vector<uint8_t>& payload, franka::CartesianVelocities& output) {
    constexpr size_t kSize = 6 * sizeof(double);

    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::CartesianVelocities>: payload size mismatch");
    }

    const uint8_t* rptr = payload.data();
    std::array<double, 6> vel{};
    decode_array_f64(rptr, vel);
    output = franka::CartesianVelocities{vel};
}

void decode(const std::vector<uint8_t>& payload, franka::Torques& output) {
    constexpr size_t kDoF = 7;
    constexpr size_t kSize = kDoF * sizeof(double);

    if (payload.size() != kSize) {
        throw std::runtime_error("decode<franka::Torques>: payload size mismatch");
    }

    const uint8_t* rptr = payload.data();
    std::array<double, kDoF> tau{};
    decode_array_f64(rptr, tau);
    output = franka::Torques{tau};
}




}  // namespace protocol
