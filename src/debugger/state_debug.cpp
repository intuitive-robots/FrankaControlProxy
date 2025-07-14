#include <iomanip>
#include <iostream>
#include "protocol/franka_arm_state.hpp"
#include "debugger/state_debug.hpp"
#include "protocol/message_header.hpp"

namespace protocol{

void debugPrintFrankaArmStateBuffer(const std::vector<uint8_t>& buffer) {
    if (buffer.size() != 652) {
        std::cerr << "Buffer size incorrect: expected " << protocol::FrankaArmState::kSize
                  << ", got " << buffer.size() << std::endl;
        return;
    }
    // Step 1: Decode header
    MessageHeader header = MessageHeader::decode(buffer.data());
    std::cout << "\n[DEBUG] MessageHeader:\n";
    std::cout << "  ID  = " << static_cast<int>(header.id) << "\n";
    std::cout << "  len = " << header.len << "\n";
    if (header.len != 636) {
        std::cerr << "[ERROR] Header length field is not 636 (actual: " << header.len << ")" << std::endl;
    }
    // Step 2: Decode FrankaArmState
    const uint8_t* payload = buffer.data() + 4;
    FrankaArmState decoded = FrankaArmState::decode(payload, 636);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n[DEBUG] FrankaArmState:\n";
    std::cout << "  timestamp_ms = " << decoded.timestamp_ms << "\n";
    auto print_array = [](const std::string& label, const auto& arr) {
        std::cout << "  " << label << " = [";
        for (size_t i = 0; i < arr.size(); ++i) {
            std::cout << arr[i];
            if (i < arr.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";
    };
    print_array("q", decoded.q);
    print_array("q_d", decoded.q_d);
    print_array("dq", decoded.dq);
    print_array("dq_d", decoded.dq_d);
    print_array("tau_ext_hat_filtered", decoded.tau_ext_hat_filtered);
    print_array("O_T_EE", decoded.O_T_EE);
    print_array("O_T_EE_d", decoded.O_T_EE_d);
    print_array("O_F_ext_hat_K", decoded.O_F_ext_hat_K);
    print_array("K_F_ext_hat_K", decoded.K_F_ext_hat_K);
}
}