#ifndef MSG_PARSER_HPP
#define MSG_PARSER_HPP
#include <vector>
#include <zmq.hpp>
#include "protocol/message_header.hpp"
#include "protocol/msg_id.hpp"

namespace protocol {
constexpr size_t HEADER_SIZE = 12; // Size of MessageHeader in bytes
class MsgParser {
public:
    static MessageHeader parse_header(const zmq::message_t& msg);
    static MsgID parse_msg_id(const zmq::message_t& msg);
    static std::vector<uint8_t> extract_payload(const zmq::message_t& msg);
    // static zmq::message_t build_message(MsgID id, const std::vector<uint8_t>& payload); // no use for now, may delete
};

}  // namespace protocol
#endif // MSG_PARSER_HPP
