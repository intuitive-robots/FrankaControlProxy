#ifndef MESSAGE_HEADER_HPP
#define MESSAGE_HEADER_HPP
#include <cstdint>
#include <cstddef>
namespace protocol {

#include <cstdint>
// TODO: shorter name like MsgHeader?
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

// TODO: I think you need a static header constructor for the encode function like following
// TODO: and the timestamp can be filled in this constructor

static MessageHeader createHeader(uint8_t message_type, uint16_t payload_length) {
    MessageHeader header{};
    header.message_type = message_type;
    header.flags = 0;
    header.payload_length = payload_length;
    header.timestamp = 0; // TODO: fill with actual timestamp
    return header;
}

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
