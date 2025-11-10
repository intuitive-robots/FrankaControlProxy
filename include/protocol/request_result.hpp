#ifndef PROTOCOL_REQUEST_RESULT_HPP
#define PROTOCOL_REQUEST_RESULT_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "protocol/msg_header.hpp"

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
    //  - flags: indicates presence of detail string
    //  - payload_length: 0, or 2 + detail.size() (u16 length prefix + bytes)
    //  - timestamp
    std::vector<uint8_t> encodeMessage() const;

    // Decode from a full message frame produced by encodeMessage()
    // Throws std::runtime_error on malformed input
    // static RequestResult decodeMessage(const std::vector<uint8_t>& frame);

private:
    RequestResultCode code_ { RequestResultCode::SUCCESS };
    std::string detail_;
};

} // namespace protocol

#endif // PROTOCOL_REQUEST_RESULT_HPP
