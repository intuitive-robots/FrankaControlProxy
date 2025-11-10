#include "protocol/request_result.hpp"
#include "protocol/codec.hpp" // encode_u16 / decode_u16

#include <cstring>
#include <limits>
#include <stdexcept>

namespace protocol {

const char* RequestResult::description(RequestResultCode code) {
    switch (code) {
        case RequestResultCode::SUCCESS:        return "Operation completed successfully";
        case RequestResultCode::FAIL:           return "Generic failure";
        case RequestResultCode::INVALID_ARG:    return "Request payload invalid or out of range";
        case RequestResultCode::BUSY:           return "Device busy / command rejected temporarily";
        case RequestResultCode::UNSUPPORTED:    return "Command not supported in current mode";
        case RequestResultCode::TIMEOUT:        return "Operation timed out";
        case RequestResultCode::COMM_ERROR:     return "Communication or CRC error";
        case RequestResultCode::INTERNAL_ERROR: return "Internal logic or hardware fault";
        default:                                return "Unknown";
    }
}
//RequestResult has it own enocdeMessage function, due to flag need to indicate presence of detail string
std::vector<uint8_t> RequestResult::encodeMessage() const {
    std::vector<uint8_t> payload;
    uint8_t flags = 0;

    if (!detail_.empty()) {
        if (detail_.size() > static_cast<size_t>(std::numeric_limits<uint16_t>::max() - 2)) {
            throw std::runtime_error("detail too long to encode");
        }
        const uint16_t len = static_cast<uint16_t>(detail_.size());
        payload.resize(2 + len);
        uint8_t* wptr = payload.data();
        encode_u16(wptr, len);
        std::memcpy(wptr, detail_.data(), len);
        flags |= FLAG_HAS_DETAIL;
    }

    MsgHeader header{};
    header.message_type   = static_cast<uint8_t>(code_);
    header.flags          = flags;
    header.payload_length = static_cast<uint16_t>(payload.size());
    header.timestamp      = 0;

    std::vector<uint8_t> frame(MsgHeader::SIZE + payload.size());
    header.encode(frame.data());
    if (!payload.empty()) {
        std::memcpy(frame.data() + MsgHeader::SIZE, payload.data(), payload.size());
    }
    return frame;
}

} // namespace protocol

