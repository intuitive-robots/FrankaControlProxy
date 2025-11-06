#include "protocol/request_result.hpp"

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

std::vector<uint8_t> RequestResult::encodeMessage() const {
    std::vector<uint8_t> payload;
    uint8_t flags = 0;

    if (!detail_.empty()) {
        if (detail_.size() > static_cast<size_t>(std::numeric_limits<uint16_t>::max() - 2)) {
            throw std::runtime_error("detail too long to encode");
        }
        const uint16_t len = static_cast<uint16_t>(detail_.size());
        payload.resize(2 + len);
        write_u16_be(payload.data(), len);
        std::memcpy(payload.data() + 2, detail_.data(), len);
        flags |= FLAG_HAS_DETAIL;
    }

    MessageHeader header{};
    header.message_type   = static_cast<uint8_t>(code_);
    header.flags          = flags;
    header.payload_length = static_cast<uint16_t>(payload.size());
    header.timestamp      = 0;

    std::vector<uint8_t> frame(MessageHeader::SIZE + payload.size());
    header.encode(frame.data());
    if (!payload.empty()) {
        std::memcpy(frame.data() + MessageHeader::SIZE, payload.data(), payload.size());
    }
    return frame;
}

// RequestResult RequestResult::decodeMessage(const std::vector<uint8_t>& frame) {
//     if (frame.size() < MessageHeader::SIZE) {
//         throw std::runtime_error("frame too small");
//     }
//     const uint8_t* data = frame.data();
//     const MessageHeader header = MessageHeader::decode(data);

//     const size_t expected = static_cast<size_t>(MessageHeader::SIZE) + header.payload_length;
//     if (frame.size() != expected) {
//         throw std::runtime_error("frame size mismatch");
//     }

//     const auto code = static_cast<RequestResultCode>(header.message_type);
//     std::string detail;
//     if (header.flags & FLAG_HAS_DETAIL) {
//         if (header.payload_length < 2) {
//             throw std::runtime_error("detail flag set but payload too small");
//         }
//         const uint8_t* payload = data + MessageHeader::SIZE;
//         const uint16_t len = read_u16_be(payload);
//         if (len != static_cast<uint16_t>(header.payload_length - 2)) {
//             throw std::runtime_error("detail length mismatch");
//         }
//         detail.assign(reinterpret_cast<const char*>(payload + 2), len);
//     } else {
//         if (header.payload_length != 0) {
//             throw std::runtime_error("payload present but detail flag not set");
//         }
//     }

//     return RequestResult{code, std::move(detail)};
// }

} // namespace protocol

