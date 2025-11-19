#pragma once
#include <zmq.hpp>

class ZmqContext {
public:
    static zmq::context_t& instance() {
        static zmq::context_t ctx{1};
        return ctx;
    }

    ZmqContext() = delete;
};