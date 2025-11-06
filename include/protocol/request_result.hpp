#ifndef PROTOCOL_REQUEST_RESULT_HPP
#define PROTOCOL_REQUEST_RESULT_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "protocol/message_header.hpp"

namespace protocol {

// Request result codes (S -> C), values follow your table (200..207)
enum class RequestResultCode : uint8_t {
    SUCCESS        = 200,
    FAIL           = 201,
    INVALID_ARG    = 202,
    BUSY           = 203,
    UNSUPPORTED    = 204,
    TIMEOUT        = 205,
    COMM_ERROR     = 206,
    INTERNAL_ERROR = 207,
};

class RequestResult {
public:
    // flag indicates payload contains a detail string
    static constexpr uint8_t FLAG_HAS_DETAIL = 0x01;

    RequestResult() = default;
    explicit RequestResult(RequestResultCode code, std::string detail = {})
        : code_(code), detail_(std::move(detail)) {}

    RequestResultCode code() const { return code_; }
    const std::string& detail() const { return detail_; }
    bool ok() const { return code_ == RequestResultCode::SUCCESS; }

    // Human-readable short description for the code
    static const char* description(RequestResultCode code);

    // Encode to a full message frame: 12-byte header + optional payload
    // Header:
    //  - message_type = static_cast<uint8_t>(code)
    //  - flags: bit0 indicates presence of detail string
    //  - payload_length: 0, or 2 + detail.size() (u16 length prefix + bytes)
    //  - timestamp: 0 (caller can patch if needed)
    std::vector<uint8_t> encodeMessage() const;

    // Decode from a full message frame produced by encodeMessage()
    // Throws std::runtime_error on malformed input
    // static RequestResult decodeMessage(const std::vector<uint8_t>& frame);

private:
    static void write_u16_be(uint8_t* dst, uint16_t v) {
        dst[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
        dst[1] = static_cast<uint8_t>(v & 0xFF);
    }
    static uint16_t read_u16_be(const uint8_t* src) {
        return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) |
                                     static_cast<uint16_t>(src[1]));
    }

private:
    RequestResultCode code_ { RequestResultCode::SUCCESS };
    std::string detail_;
};

} // namespace protocol

#endif // PROTOCOL_REQUEST_RESULT_HPP

