#pragma once

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <functional>
#include <iostream>
#include <string>
#include <cstdint>

#include <flatbuffers/flatbuffers.h>

class ServiceRegistry {
public:
    // Result code for the first reply frame
    enum class ResultCode : std::uint8_t {
        Success    = 0,
        Failure    = 1,
        NoService  = 2,
        BadRequest = 3
    };

    // Low-level handler type: gets raw FlatBuffers bytes, returns FlatBuffers buffer
    using HandlerFn = std::function<flatbuffers::DetachedBuffer(const std::uint8_t* data, std::size_t size)>;

    ServiceRegistry(zmq::context_t& context, const std::string& endpoint)
        : context_(context),
          res_socket_(context, zmq::socket_type::rep),
          endpoint_(endpoint),
          running_(false)
    {
        res_socket_.bind(endpoint_);
    }

    ~ServiceRegistry() {
        stop();
    }

    // Start worker thread and bind res socket to endpoint
    void start() {
        if (running_) return;
        running_ = true;
        service_thread_ = std::thread(&ServiceRegistry::serviceLoop, this);
        std::cout << "[ServiceRegistry] res listening on " << endpoint_ << std::endl;
    }

    // Stop worker thread and close socket
    void stop() {
        if (!running_) return;

        running_ = false;

        // Closing the socket will unblock a blocking recv()
        try {
            res_socket_.close();
        } catch (const zmq::error_t& e) {
            std::cerr << "[ServiceRegistry] Error closing socket: "
                      << e.what() << std::endl;
        }

        if (service_thread_.joinable()) {
            service_thread_.join();
        }
    }

    // Register handler using enum-like message ID
    template<typename ClassT, typename RequestT, typename ResponseT>
    void registerHandler(std::string service_name,
                        ClassT* instance,
                        ResponseT (ClassT::*method)(const RequestT&))
    {
        handlers_[service_name] =
            [instance, method](const uint8_t* data, size_t size)
            -> flatbuffers::DetachedBuffer 
        {
            (void)size;
            const RequestT* req = flatbuffers::GetRoot<RequestT>(data);
            ResponseT resp = (instance->*method)(*req);

            flatbuffers::FlatBufferBuilder builder;
            builder.Finish(resp);
            return builder.Release();
        };
    }

    // ---------------------------------------------------------------
    // REGISTER SERVICE WITHOUT REQUEST TYPE
    //
    // - ZMQ only sends one frame: [ MsgID ]
    // - No FlatBuffers payload
    // - Callback signature:
    //       ResponseT (ClassT::*)()
    //
    // ---------------------------------------------------------------
    template<typename ClassT, typename ResponseT>
    void registerHandler(std::string service_name,
                        ClassT* instance,
                        ResponseT (ClassT::*method)())
    {
        handlers_[service_name] =
            [instance, method]()
            -> flatbuffers::DetachedBuffer 
        {
            ResponseT resp = (instance->*method)();
            flatbuffers::FlatBufferBuilder builder;
            auto fb = CreateResponseT(builder, resp);
            builder.Finish(fb);
            return builder.Release();
        };
    }


private:
    zmq::context_t& context_;
    zmq::socket_t   res_socket_;
    std::string     endpoint_;

    std::atomic<bool> running_;
    std::thread       service_thread_;

    std::unordered_map<std::string, HandlerFn> handlers_;

    // Main service loop running in a background thread
    void serviceLoop() {
        try {
            while (running_) {
                zmq::message_t service_name_msg;
                zmq::message_t request_msg;

                // Receive two-part message:
                //   [0] service name (string)
                //   [1] FlatBuffers payload
                if (!res_socket_.recv(service_name_msg, zmq::recv_flags::none)) {
                    if (!running_) break;
                    continue;
                }

                if (!res_socket_.recv(request_msg, zmq::recv_flags::none)) {
                    if (!running_) break;
                    continue;
                }
                std::string service_name(
                    static_cast<const char*>(service_name_msg.data()),
                    service_name_msg.size());

                const std::uint8_t* payload_ptr =
                    static_cast<const std::uint8_t*>(request_msg.data());
                std::size_t payload_size = request_msg.size();

                std::cout << "[ServiceRegistry] Request for service: "
                          << service_name << std::endl;

                ResultCode result_code = ResultCode::Success;
                flatbuffers::DetachedBuffer reply_buf;

                auto it = handlers_.find(service_name);
                if (it == handlers_.end()) {
                    result_code = ResultCode::NoService;
                    reply_buf = flatbuffers::DetachedBuffer(); // empty response
                } else {
                    try {
                        reply_buf = it->second(payload_ptr, payload_size);
                        result_code = ResultCode::Success;
                    } catch (const std::exception& e) {
                        std::cerr << "[ServiceRegistry] Handler exception: "
                                  << e.what() << std::endl;
                        result_code = ResultCode::Failure;
                        reply_buf = flatbuffers::DetachedBuffer(); // empty
                    } catch (...) {
                        std::cerr << "[ServiceRegistry] Unknown handler exception"
                                  << std::endl;
                        result_code = ResultCode::Failure;
                        reply_buf = flatbuffers::DetachedBuffer(); // empty
                    }
                }

                // Send reply:
                //   [0] result code (single byte)
                //   [1] FlatBuffers serialized response (may be empty)
                zmq::message_t code_msg(sizeof(std::uint8_t));
                *static_cast<std::uint8_t*>(code_msg.data()) =
                    static_cast<std::uint8_t>(result_code);

                zmq::message_t data_msg(reply_buf.size());
                if (reply_buf.size() > 0) {
                    std::memcpy(data_msg.data(), reply_buf.data(), reply_buf.size());
                }

                res_socket_.send(code_msg, zmq::send_flags::sndmore);
                res_socket_.send(data_msg, zmq::send_flags::none);
            }
        } catch (const zmq::error_t& e) {
            // It is normal to get an exception here when socket is closed on stop()
            if (running_) {
                std::cerr << "[ServiceRegistry] ZMQ error in serviceLoop: "
                          << e.what() << std::endl;
            }
        }
    }
};
