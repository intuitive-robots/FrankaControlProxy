#include "protocol/msg_parser.hpp"
#include <cstring>
//when use this, remember to use try_catch to catch std::runtime_error
namespace protocol {

MessageHeader MsgParser::parse_header(const zmq::message_t& msg) {
    if (msg.size() < HEADER_SIZE) {
        throw std::runtime_error("Message too small");
    }
    return MessageHeader::decode(static_cast<const uint8_t*>(msg.data()));
}//returen a MessageHeader object

MsgID MsgParser::parse_msg_id(const zmq::message_t& msg) {
    return static_cast<MsgID>(parse_header(msg).id);
}//return a enumerator in MsgID

std::vector<uint8_t> MsgParser::extract_payload(const zmq::message_t& msg) {
    if (msg.size() < HEADER_SIZE) {
        throw std::runtime_error("No payload in message");
    }
    const uint8_t* start = static_cast<const uint8_t*>(msg.data()) + HEADER_SIZE;
    return std::vector<uint8_t>(start, start + msg.size() - HEADER_SIZE);
}//return a vector of uint8_t, the payload of the message


//no use for now, may delete
// zmq::message_t MsgParser::build_message(MsgID id, const std::vector<uint8_t>& payload) {
//     MessageHeader header;
//     header.id = static_cast<uint8_t>(id);
//     header.len = static_cast<uint16_t>(payload.size());
//     header.pad = 0;

//     size_t total_size = HEADER_SIZE + payload.size();
//     zmq::message_t msg(total_size);

//     // encode header
//     header.encode(static_cast<uint8_t*>(msg.data()));
//     // copy payload
//     std::memcpy(static_cast<uint8_t*>(msg.data()) + HEADER_SIZE, payload.data(), payload.size());
//     // zero-fill padding
//     std::memset(static_cast<uint8_t*>(msg.data()) + total_size, 0, msg.size() - total_size);

//     return msg;
// }

}  // namespace protocol
