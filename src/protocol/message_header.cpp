#include "protocol/message_header.hpp"

namespace protocol {

// Encode 12-byte header in network byte order (big-endian)
void MessageHeader::encode(uint8_t* buffer) const {
    buffer[0] = message_type;

    buffer[1] = flags;

    buffer[2] = static_cast<uint8_t>((payload_length >> 8) & 0xFF); // high byte
    buffer[3] = static_cast<uint8_t>(payload_length & 0xFF);        // low byte

    const uint64_t ts = timestamp;
    buffer[4]  = static_cast<uint8_t>((ts >> 56) & 0xFF);
    buffer[5]  = static_cast<uint8_t>((ts >> 48) & 0xFF);
    buffer[6]  = static_cast<uint8_t>((ts >> 40) & 0xFF);
    buffer[7]  = static_cast<uint8_t>((ts >> 32) & 0xFF);
    buffer[8]  = static_cast<uint8_t>((ts >> 24) & 0xFF);
    buffer[9]  = static_cast<uint8_t>((ts >> 16) & 0xFF);
    buffer[10] = static_cast<uint8_t>((ts >> 8)  & 0xFF);
    buffer[11] = static_cast<uint8_t>(ts & 0xFF);
}

// Decode 12-byte header from network byte order (big-endian)
MessageHeader MessageHeader::decode(const uint8_t* buffer) {
    MessageHeader header{};
    header.message_type   = buffer[0];

    header.flags          = buffer[1];
    
    header.payload_length = static_cast<uint16_t>(
        (static_cast<uint16_t>(buffer[2]) << 8) |
         static_cast<uint16_t>(buffer[3])
    );

    uint64_t ts = 0;
    ts |= static_cast<uint64_t>(buffer[4])  << 56;
    ts |= static_cast<uint64_t>(buffer[5])  << 48;
    ts |= static_cast<uint64_t>(buffer[6])  << 40;
    ts |= static_cast<uint64_t>(buffer[7])  << 32;
    ts |= static_cast<uint64_t>(buffer[8])  << 24;
    ts |= static_cast<uint64_t>(buffer[9])  << 16;
    ts |= static_cast<uint64_t>(buffer[10]) << 8;
    ts |= static_cast<uint64_t>(buffer[11]);
    header.timestamp = ts;

    return header;
}

}  // namespace protocol
