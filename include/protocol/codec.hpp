#pragma once
#include "protocol/byte_order.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/mode_id.hpp"
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>
#include <franka/robot_state.h>
#include <franka/model.h>

namespace protocol {

class RequestResult; // forward declaration?

// encode std::array<double, N> (fixed size) 
template <size_t N>
inline void encode_array_f64(uint8_t*& ptr, const std::array<double, N>& in) {
    for (size_t i = 0; i < N; ++i) {
        double be_val = to_big_endian_f64(in[i]);
        std::memcpy(ptr, &be_val, sizeof(double));
        ptr += sizeof(double);
    }
}

// encoede std::vector<double> (dynamic size) 
inline void encode_array_f64(uint8_t*& ptr, const std::vector<double>& in) {
    for (const auto& val : in) {
        double be_val = to_big_endian_f64(val);
        std::memcpy(ptr, &be_val, sizeof(double));
        ptr += sizeof(double);
    }
}

// decode std::array<double, N> (fixed size)
template <size_t N>
inline void decode_array_f64(const uint8_t*& ptr, std::array<double, N>& out) {
    for (size_t i = 0; i < N; ++i) {
        double raw;
        std::memcpy(&raw, ptr, sizeof(double));
        out[i] = from_big_endian_f64(raw);
        ptr += sizeof(double);
    }
}

// decode std::vector<double>（dynamic size）
inline void decode_array_f64(const uint8_t*& ptr, std::vector<double>& out, size_t count) {
    out.resize(count);
    for (size_t i = 0; i < count; ++i) {
        double raw;
        std::memcpy(&raw, ptr, sizeof(double));
        out[i] = from_big_endian_f64(raw);
        ptr += sizeof(double);
    }
}

// encode uint32_t
inline void encode_u32(uint8_t*& ptr, uint32_t val) {
    uint32_t be_val = to_big_endian_u32(val);
    std::memcpy(ptr, &be_val, sizeof(be_val));
    ptr += sizeof(be_val);
}

//encode uint16_t
inline void encode_u16(uint8_t*& ptr, uint16_t val) {
    uint16_t be_val = to_big_endian_u16(val);
    std::memcpy(ptr, &be_val, sizeof(be_val));
    ptr += sizeof(be_val);
}


// decode uint32_t
inline uint32_t decode_u32(const uint8_t*& ptr) {
    uint32_t raw;
    std::memcpy(&raw, ptr, sizeof(raw));
    ptr += sizeof(raw);
    return from_big_endian_u32(raw);
}

// decode uint16_t
inline uint16_t decode_u16(const uint8_t*& ptr) {
    uint16_t raw;
    std::memcpy(&raw, ptr, sizeof(raw));
    ptr += sizeof(raw);
    return from_big_endian_u16(raw);
}

//encode double
inline void encode_f64(uint8_t*& ptr, double val) {
    double be_val = to_big_endian_f64(val);
    std::memcpy(ptr, &be_val, sizeof(be_val));
    ptr += sizeof(be_val);
}
// decode double
inline double decode_f64(const uint8_t*& ptr) {
    double raw;
    std::memcpy(&raw, ptr, sizeof(raw));
    ptr += sizeof(raw);
    return from_big_endian_f64(raw);
}

// encode bool
inline void encode_bool(uint8_t*& ptr, bool val) {
    uint8_t byte_val = val ? 1 : 0; // Convert bool to uint8_t
    std::memcpy(ptr, &byte_val, sizeof(byte_val));
    ptr += sizeof(byte_val);
}  

// decode bool
inline bool decode_bool(const uint8_t*& ptr) {
    uint8_t byte_val;
    std::memcpy(&byte_val, ptr, sizeof(byte_val));
    ptr += sizeof(byte_val);
    return byte_val != 0; // Convert uint8_t back to bool
}

// ------------------------------------------------------------
// Generic payload-level codec (overloads + template decode)
// Note: These operate on payload only (no 12-byte header)
// ------------------------------------------------------------

// encode overloads by argument type (valid C++ overloading)
std::vector<uint8_t> encode(const std::string& v);
std::vector<uint8_t> encode(uint8_t v);
std::vector<uint8_t> encode(uint16_t v);
std::vector<uint8_t> encode(const franka::RobotState& rs);
//RequestResult has it own enocdeMessage function, due to flag need to indicate presence of detail string


// explicit specializations declarations
void decode(const std::vector<uint8_t>& payload, std::string& output);
void decode(const std::vector<uint8_t>& payload, uint8_t& output);
void decode(const std::vector<uint8_t>& payload, uint16_t& output);

// libfranka control types
// payload-only encoder extracting required fields from full RobotState
void decode(const std::vector<uint8_t>& payload, franka::JointPositions& output);
void decode(const std::vector<uint8_t>& payload, franka::CartesianPose& output);
void decode(const std::vector<uint8_t>& payload, franka::CartesianVelocities& output);
void decode(const std::vector<uint8_t>& payload, franka::JointVelocities& output);
void decode(const std::vector<uint8_t>& payload, franka::Torques& output);

}  // namespace protocol

