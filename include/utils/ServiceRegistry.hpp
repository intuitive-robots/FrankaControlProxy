// ServiceRegistry.h
#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include "protocol/codec.hpp"
#include "protocol/message_header.hpp"

template <typename ClassType, typename HeaderType>
class ServiceRegistry {
public:
    using RawHandler = std::function<void(ClassType*, const HeaderType&, const uint8_t*, std::vector<uint8_t>&)>;

    void registerRawHandler(int msg_id, RawHandler fn) {
        handlers_[msg_id] = std::move(fn);
    }

    template <typename Req, typename Resp>
    void registerHandler(int msg_id, Resp (ClassType::*method)(Req)) {
        handlers_[msg_id] = [method](ClassType* self, const HeaderType& header,
                                     const uint8_t* payload, std::vector<uint8_t>& resp_buf) {
            // 1. decode payload → Req
            Req req = protocol::decode<Req>(payload);
            // 2. call member function
            Resp resp_obj = (self->*method)(req);
            // 3. encode Resp → bytes
            resp_buf = protocol::encode(resp_obj);
        };
    }

    bool dispatch(ClassType* instance, int msg_id,
                  const HeaderType& header, const uint8_t* payload,
                  std::vector<uint8_t>& response) const {
        auto it = handlers_.find(msg_id);
        if (it != handlers_.end()) {
            it->second(instance, header, payload, response);
            return true;
        }
        return false;
    }

    // TODO: implement this
    void handleMessage(protocol::MessageHeader header,
                       const std::vector<uint8_t>& payload,
                       std::vector<uint8_t>& response) const {
        if (message.size() < protocol::MessageHeader::SIZE) {
            throw std::runtime_error("Message too short");
        }
    }


private:
    std::unordered_map<int, RawHandler> handlers_;
};
