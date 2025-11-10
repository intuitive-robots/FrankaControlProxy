#pragma once
#include <cstdint>
#include <cstddef>
#include <chrono>
namespace protocol {

struct MsgHeader {
    uint8_t  message_type;    // Data type (see MsgID table) - uint8
    uint8_t  flags;           // Bit flags for message options   - uint8
    uint16_t payload_length;  // Payload length in bytes         - uint16
    uint64_t timestamp;       // High-precision timestamp (ns)   - uint64

    // Serialized header size: 1 + 1 + 2 + 8 = 12 bytes
    static constexpr size_t SIZE = 12;

    void encode(uint8_t* buffer) const;// using createHeader before, write header to buffer
    
    static MsgHeader decode(const uint8_t* buffer);
};

static MsgHeader createHeader(uint8_t message_type, uint16_t payload_length) {
    MsgHeader header{};
    header.message_type = message_type;//MsgID
    header.flags = 0;
    header.payload_length = payload_length;
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    header.timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
    return header;
}


}  // namespace protocol
