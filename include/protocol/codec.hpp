#ifndef CODEC_HPP
#define CODEC_HPP
#include "protocol/byte_order.hpp"
#include "protocol/message_header.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/franka_arm_state.hpp"
#include "protocol/franka_gripper_state.hpp"
#include "protocol/mode_id.hpp"
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>
#include <franka/robot_state.h>

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

// // TODO: could you use overload function for encodeMessage? like encode for all the message type
// std::vector<uint8_t> encodeMessage(const MsgHeader& header, const std::vector<uint8_t>& payload);
// std::vector<uint8_t> encodeStateMessage(const FrankaArmState& state);
// std::vector<uint8_t> encodeModeMessage(uint8_t mode_code);
// //Todo:check if need vector of following 2 functions
// std::vector<uint8_t> encodeRequestResultMessage(const RequestResult& result); // Todo: implement RequestResult class
// std::vector<uint8_t> encodePubPortMessage(uint16_t Pubport);//Todo:implement, and whta Pubport old name
// //Todo:check if return bool
// // TODO: also the same for decode
// bool decodeModeMessage(const std::vector<uint8_t>& data, uint8_t& mode_code);//Todo:change into FrankaArmControl
// bool decodeCommandMessage(const std::vector<uint8_t>& data, uint8_t& command);
// //Todo: think of the exact command class name? check with the header 11 for Command
// //Than the exact type of commmand should be switch by the following 0-5



// ------------------------------------------------------------
// Generic payload-level codec (overloads + template decode)
// Note: These operate on payload only (no 12-byte header)
// ------------------------------------------------------------

// encode overloads by argument type (valid C++ overloading)
std::vector<uint8_t> encode(const std::string& v);
std::vector<uint8_t> encode(uint8_t v);
std::vector<uint8_t> encode(uint16_t v);

// Response helper: minimal payload encoding for RequestResult (1-byte code)
// If you need detail string, extend to include length + bytes and set header.flags outside.
std::vector<uint8_t> encode(const RequestResult& v);

// decode must be template, since return type alone cannot overload in C++
template <typename T>
T decode(const std::vector<uint8_t>& payload);

// explicit specializations declarations
template <>
std::string decode<std::string>(const std::vector<uint8_t>& payload);

template <>
uint8_t decode<uint8_t>(const std::vector<uint8_t>& payload);

template <>
uint16_t decode<uint16_t>(const std::vector<uint8_t>& payload);


// libfranka control types
// payload-only encoder extracting required fields from full RobotState
std::vector<uint8_t> encode(const franka::RobotState& rs);

template <>
franka::JointPositions decode<franka::JointPositions>(const std::vector<uint8_t>& payload);

template <>
franka::CartesianPose decode<franka::CartesianPose>(const std::vector<uint8_t>& payload);

template <>
franka::CartesianVelocities decode<franka::CartesianVelocities>(const std::vector<uint8_t>& payload);

template <>
franka::JointVelocities decode<franka::JointVelocities>(const std::vector<uint8_t>& payload);




}  // namespace protocol
#endif // CODEC_HPP
