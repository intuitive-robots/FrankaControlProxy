#ifndef MESSAGE_HEADER_HPP
#define MESSAGE_HEADER_HPP
#include <cstdint>
#include <cstddef>
namespace protocol {

#include <cstdint>

struct MessageHeader {
    uint8_t  message_type;    // Data type (see MsgID table) - uint8
    uint8_t  flags;           // Bit flags for message options   - uint8
    uint16_t payload_length;  // Payload length in bytes         - uint16
    uint64_t timestamp;       // High-precision timestamp (ns)   - uint64

    // Serialized header size: 1 + 1 + 2 + 8 = 12 bytes
    static constexpr size_t SIZE = 12;

    void encode(uint8_t* buffer) const;
    
    static MessageHeader decode(const uint8_t* buffer);
};

// struct MessageHeader {
//     uint8_t id;     // Message ID (1 byte)
//     uint16_t len;   // Payload length (2 bytes)
//     uint8_t pad = 0;// Always 0 (1 byte)

//     static constexpr size_t SIZE = 4;

   
//     void encode(uint8_t* buffer) const;

  
//     static MessageHeader decode(const uint8_t* buffer);
// };

}  // namespace protocol
#endif // MESSAGE_HEADER_HPP
