// ServiceRegistry.h
#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include "protocol/codec.hpp"
#include "protocol/msg_header.hpp"
#include "protocol/msg_id.hpp"
#include "protocol/request_result.hpp"


class ServiceRegistry {
public:
    template <typename ClassT, typename RequestType, typename ResponseType>
    void registerHandler(const protocol::MsgID& name, ClassT* instance, ResponseType (ClassT::*method)(RequestType)) {
        handlers_[name] = [instance, method](const std::vector<uint8_t>& payload) -> std::vector<uint8_t> {
            // Decode payload into RequestType via template decode<T>
            RequestType arg = protocol::decode<RequestType>(payload);
            // Invoke bound member function
            ResponseType ret = (instance->*method)(arg);
            // Encode ResponseType back to payload bytes
            return protocol::encode(ret);
        };
    }

    std::vector<uint8_t> handleMessage(const protocol::MsgHeader& header, const std::vector<uint8_t>& payload) {
        auto it = handlers_.find(static_cast<protocol::MsgID>(header.message_type));
        if (it == handlers_.end()) {
            // Encode FAIL code + detail string payload (u16 len + bytes)
            const std::string err = "Unknown handler";
            protocol::RequestResult rr(protocol::RequestResultCode::FAIL, err);
            std::vector<uint8_t> out = protocol::encode(rr);          // 1-byte code
            std::vector<uint8_t> detail = protocol::encode(err);       // 2+N bytes
            out.insert(out.end(), detail.begin(), detail.end());
            return out;
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

