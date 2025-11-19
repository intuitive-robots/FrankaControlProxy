// ServiceRegistry.h
#pragma once
#include <unordered_map>
#include <functional>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <iostream>
#include "protocol/codec.hpp"
#include "protocol/request_result.hpp"
#include "protocol/request_result.hpp"
#include "utils/zmq_context.hpp"


class ServiceRegistry {
public:

    ServiceRegistry()
        : res_socket_(ZmqContext::instance(), zmq::socket_type::rep) {
        res_socket_.set(zmq::sockopt::rcvtimeo, SOCKET_TIMEOUT_MS);
    };

    ~ServiceRegistry() {
        stop();
        res_socket_.close();
    }

    void bindSocket(const std::string& service_addr) {
        service_addr_ = service_addr;
        res_socket_.bind(service_addr_);
    }

    void start() {
        // check res_socket_ is binded?
        if (service_addr_.empty()) {
            throw std::runtime_error("ServiceRegistry socket not bound. Call bindSocket() before start().");
        }
        is_running = true;
        service_thread_ = std::thread(&ServiceRegistry::responseSocketThread, this);
    }

    void stop() {
        is_running = false;
        if (service_thread_.joinable()) {
            service_thread_.join();
        }
    }

    template <typename ClassT, typename RequestType, typename ResponseType>
    void registerHandler(const std::string& name, ClassT* instance, ResponseType (ClassT::*method)(const RequestType&)) {
        handlers_[name] = [instance, method](const protocol::ByteView& payload) -> std::vector<uint8_t> {
            // Decode payload into RequestType via template decode<T>
            RequestType arg = protocol::decode<RequestType>(payload);
            // Invoke bound member function
            ResponseType ret = (instance->*method)(arg);
            // Encode ResponseType back to payload bytes
            return protocol::encode(ret);
        };
    }


    template <typename ClassT, typename ResponseType>
    void registerHandler(const std::string& name, ClassT* instance,
                        ResponseType (ClassT::*method)()) {
        handlers_[name] = [instance, method](const protocol::ByteView&) -> std::vector<uint8_t> {
            ResponseType ret = (instance->*method)();
            return protocol::encode(ret);
        };
    }

    // method: void func(const RequestType&)
    template <typename ClassT, typename RequestType>
    void registerHandler(const std::string& name,
                        ClassT* instance,
                        void (ClassT::*method)(const RequestType&)) {

        handlers_[name] = [instance, method](const protocol::ByteView& payload)
            -> std::vector<uint8_t>
        {
            RequestType arg = protocol::decode<RequestType>(payload);
            (instance->*method)(arg);
            return {};
        };
    }


    void handleRequest(const std::string& service_name, const protocol::ByteView& payload, protocol::FrankaResponse& response) {
        auto it = handlers_.find(service_name);
        std::cout << "[ServiceRegistry] Handling message of type " << service_name << std::endl;
        if (it == handlers_.end()) {
            // const std::string err = "Unknown handler";
            // protocol::FrankaResponse rr(protocol::RequestResultCode::FAIL, err);
            // std::vector<uint8_t> out = protocol::encode(rr);
            // std::vector<uint8_t> detail = protocol::encode(err);
            // out.insert(out.end(), detail.begin(), detail.end());
            // response = std::move(out);
            // result_code = protocol::RequestResultCode::FAIL;
            return;
        }
        response.code = protocol::FrankaResponseCode::SUCCESS;
        try {
            response.payload = it->second(payload);
        } catch (const std::exception& e) {
            std::cerr << "[ServiceRegistry] Exception while handling " << service_name << " service request: " << e.what() << std::endl;
            response.code = protocol::FrankaResponseCode::FAIL;
        }
        std::cout << "[ServiceRegistry] Found handler for service " << service_name << std::endl;
    }

    void clearHandlers() {
        handlers_.clear();
    }

    void removeHandler(const std::string& name) {
        handlers_.erase(name);
    }

        //response socket thread, used for service request
    void responseSocketThread() {
        while (is_running) {
            zmq::message_t service_name_msg;
            if (!res_socket_.recv(service_name_msg, zmq::recv_flags::none)) continue;
            std::cout << "[FrankaArmProxy] Received service request message of size " << service_name_msg.size() << " bytes." << std::endl;
            std::string service_name = protocol::decode<std::string>(protocol::ByteView{
                static_cast<const uint8_t*>(service_name_msg.data()),
                service_name_msg.size()
            });


            if (!service_name_msg.more()) {
                std::cerr << "[FrankaArmProxy] Warning: No payload frame received for service request." << std::endl;
                continue; // Skip this iteration if no payload
            }
            zmq::message_t payload_msg;
            if (!res_socket_.recv(payload_msg, zmq::recv_flags::none)) continue;
            std::cout << "[FrankaArmProxy] Received payload message of size " << payload_msg.size() << " bytes." << std::endl;
            protocol::ByteView payload{
                static_cast<const uint8_t*>(payload_msg.data()),
                payload_msg.size()
            };
            if (payload_msg.more()) {
                std::cerr << "[FrankaArmProxy] Warning: More message frames received than expected." << std::endl;
            }
            std::cout << "[FrankaArmProxy] Received request " << service_name << std::endl;
            //std::string response;
            protocol::FrankaResponse response;
            handleRequest(service_name, payload, response);
            //send response
            res_socket_.send(zmq::buffer(service_name), zmq::send_flags::sndmore);
            res_socket_.send(zmq::buffer(response.payload), zmq::send_flags::none);
            std::cout << "[FrankaArmProxy] Sent response: msg size = " << response.payload.size() << std::endl;
        }
    }

        ServiceRegistry(const ServiceRegistry&) = delete;
        ServiceRegistry& operator=(const ServiceRegistry&) = delete;

        ServiceRegistry(ServiceRegistry&&) = default;
        ServiceRegistry& operator=(ServiceRegistry&&) = default;


    private:
        std::unordered_map<std::string, std::function<std::vector<uint8_t>(const protocol::ByteView&)>> handlers_;
        bool is_running{false};
        zmq::socket_t res_socket_{ZmqContext::instance(), zmq::socket_type::rep};
        static constexpr int SOCKET_TIMEOUT_MS = 100;
        std::thread service_thread_;
        std::string service_addr_;
    };