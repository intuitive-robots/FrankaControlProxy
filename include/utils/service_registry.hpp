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
#include "protocol/msg_id.hpp"


class ServiceRegistry {
public:
    template <typename ClassT, typename RequestType, typename ResponseType>
    void registerHandler(const protocol::MsgID& name, ClassT* instance, ResponseType (ClassT::*method)(RequestType)) {
        handlers_[name] = [instance, method](const std::vector<uint8_t>& payload) -> std::vector<uint8_t> {
            // TODO: I need somthing like this decoder from protocol
            RequestType arg = protocol::decode(payload);
            ResponseType ret = (instance->*method)(arg);
            // TODO: I need something like this encoder from protocol
            return protocol::encode(ret);
        };
    }

    std::vector<uint8_t> handleMessage(const protocol::MessageHeader& header, const std::vector<uint8_t>& payload) {
        auto it = handlers_.find(static_cast<protocol::MsgID>(header.message_type));
        if (it == handlers_.end()) {
            std::string err = "Unknown handler";
            return {err.begin(), err.end()};
        }
        return it->second(payload);
    }

    void clearHandlers() {
        handlers_.clear();
    }

    void removeHandler(const protocol::MsgID& name) {
        handlers_.erase(name);
    }

private:
    std::unordered_map<
        protocol::MsgID,
        std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>
    > handlers_;
};

