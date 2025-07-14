#include "protocol/franka_arm_state.hpp"
#include "protocol/byte_order.hpp"
#include "protocol/codec.hpp"
#include <cstring>
#include <stdexcept>
#include <chrono>
namespace protocol {

std::vector<uint8_t> FrankaArmState::encode() const {
    std::vector<uint8_t> buffer(kSize);
    uint8_t* ptr = buffer.data();

    encode_u32(ptr, timestamp_ms);
    encode_array_f64(ptr, O_T_EE);
    encode_array_f64(ptr, O_T_EE_d);
    encode_array_f64(ptr, q);
    encode_array_f64(ptr, q_d);
    encode_array_f64(ptr, dq);
    encode_array_f64(ptr, dq_d);
    encode_array_f64(ptr, tau_ext_hat_filtered);
    encode_array_f64(ptr, O_F_ext_hat_K);
    encode_array_f64(ptr, K_F_ext_hat_K);

    return buffer;
}

FrankaArmState FrankaArmState::decode(const uint8_t* buffer, size_t size) {
    if (size != kSize) {
        throw std::runtime_error("FrankaArmState::decode() size mismatch");
    }

    FrankaArmState state;
    const uint8_t* ptr = buffer;

    state.timestamp_ms = decode_u32(ptr);
    decode_array_f64(ptr, state.O_T_EE);
    decode_array_f64(ptr, state.O_T_EE_d);
    decode_array_f64(ptr, state.q);
    decode_array_f64(ptr, state.q_d);
    decode_array_f64(ptr, state.dq);
    decode_array_f64(ptr, state.dq_d);
    decode_array_f64(ptr, state.tau_ext_hat_filtered);
    decode_array_f64(ptr, state.O_F_ext_hat_K);
    decode_array_f64(ptr, state.K_F_ext_hat_K);

    return state;
}

FrankaArmState FrankaArmState::fromRobotState(const franka::RobotState& rs) {
    FrankaArmState state;
    state.timestamp_ms = static_cast<uint32_t>(rs.time.toMSec());//get in ms
    std::copy(rs.q.begin(), rs.q.end(), state.q.begin());
    std::copy(rs.q_d.begin(), rs.q_d.end(), state.q_d.begin());
    std::copy(rs.dq.begin(), rs.dq.end(), state.dq.begin());
    std::copy(rs.dq_d.begin(), rs.dq_d.end(), state.dq_d.begin());
    std::copy(rs.tau_ext_hat_filtered.begin(), rs.tau_ext_hat_filtered.end(), state.tau_ext_hat_filtered.begin());

    std::copy(rs.O_T_EE.begin(), rs.O_T_EE.end(), state.O_T_EE.begin());
    std::copy(rs.O_T_EE_d.begin(), rs.O_T_EE_d.end(), state.O_T_EE_d.begin());
    std::copy(rs.O_F_ext_hat_K.begin(), rs.O_F_ext_hat_K.end(), state.O_F_ext_hat_K.begin());
    std::copy(rs.K_F_ext_hat_K.begin(), rs.K_F_ext_hat_K.end(), state.K_F_ext_hat_K.begin());

    return state;
}

franka::RobotState FrankaArmState::toRobotState() const {
    franka::RobotState rs;

    // transfer timestamp(need to test)
    rs.time = franka::Duration(timestamp_ms * 1000000); // ms → ns

    std::copy(q.begin(), q.end(), rs.q.begin());
    std::copy(q_d.begin(), q_d.end(), rs.q_d.begin());
    std::copy(dq.begin(), dq.end(), rs.dq.begin());
    std::copy(dq_d.begin(), dq_d.end(), rs.dq_d.begin());
    std::copy(tau_ext_hat_filtered.begin(), tau_ext_hat_filtered.end(), rs.tau_ext_hat_filtered.begin());

    std::copy(O_T_EE.begin(), O_T_EE.end(), rs.O_T_EE.begin());
    std::copy(O_T_EE_d.begin(), O_T_EE_d.end(), rs.O_T_EE_d.begin());
    std::copy(O_F_ext_hat_K.begin(), O_F_ext_hat_K.end(), rs.O_F_ext_hat_K.begin());
    std::copy(K_F_ext_hat_K.begin(), K_F_ext_hat_K.end(), rs.K_F_ext_hat_K.begin());

    return rs;
}


}  // namespace protocol
