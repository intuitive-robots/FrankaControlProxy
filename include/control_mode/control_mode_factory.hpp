#pragma once
#include "control_mode/abstract_control_mode.hpp"
#include <unordered_map>
#include <string>
#include <functional>


class ControlModeFactory {
public:

    static void registerMode(const std::string& name, std::function<std::shared_ptr<AbstractControlMode>()> creator) {
        std::cout << "[ControlModeFactory] Registering mode: " << name << std::endl;
        getRegistry()[name] = std::move(creator);
    }

    static std::shared_ptr<AbstractControlMode> create(const std::string& name) {
        auto& reg = getRegistry();
        if (auto it = reg.find(name); it != reg.end())
            return it->second();
        throw std::runtime_error("Unknown mode: " + name);
    }

private:
    static std::unordered_map<std::string, std::function<std::shared_ptr<AbstractControlMode>()>>& getRegistry() {
        static std::unordered_map<std::string, std::function<std::shared_ptr<AbstractControlMode>()>> instance;
        return instance;
    }
};
