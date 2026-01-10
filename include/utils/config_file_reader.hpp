#pragma once
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <zerolancom/zerolancom.hpp>

#include "utils/exceptions.hpp"

class ConfigFileReader
{
  public:
    ConfigFileReader() = delete;
    virtual ~ConfigFileReader() = default;

    ConfigFileReader(const std::string& arm_config_path)
    {
        try
        {
            node = YAML::LoadFile(arm_config_path);
        }
        catch (const YAML::BadFile& e)
        {
            zlc::error("Error: Could not open config file: {} ({})", arm_config_path, e.what());
        }
        catch (const std::exception& e)
        {
            zlc::error("Error: Failed to parse config: {}", e.what());
        }
    }

    template <typename T>
    T getValue(const std::string& key) const
    {
        if (!node[key])
        {
            throw ConfigValueNotFound(key);
        }
        try
        {
            return node[key].as<T>();
        }
        catch (const std::exception& e)
        {
            zlc::warn("Config parse warning for key {}: {}", key, e.what());
            throw e;
        }
    }

    template <typename T, size_t N>
    std::array<T, N> getArray(const std::string& key) const
    {
        if (!node[key] || !node[key].IsSequence())
        {
            throw ConfigValueNotFound(key);
        }
        try
        {
            auto vec = node[key].as<std::vector<double>>();
            if (vec.size() != N)
            {
                zlc::warn("Config key {} expected {} values, got {}. Using fallback.", key, N,
                          vec.size());
                throw ConfigValueNotFound(key);
            }
            std::array<T, N> arr{};
            std::copy(vec.begin(), vec.end(), arr.begin());
            return arr;
        }
        catch (const std::exception& e)
        {
            zlc::warn("Config parse warning for key {}: {}", key, e.what());
            throw e;
        }
    }

  private:
    YAML::Node node;
};

struct FrankaGripperConfig
{
    // communication
    std::string gripper_ip;
    std::string gripper_state_pub_addr;
    std::string gripper_service_addr;

    // gripper
    int gripper_pub_rate_hz{100};
    int gripper_command_rcvtimeo_ms{500};
    double gripper_default_close_open_threshold{0.01};
    double gripper_default_speed_slow{0.01};
    double gripper_default_speed_fast{20.0};
    double gripper_default_force{20.0};

    void fromFile(const std::string& gripper_config_path)
    {
        ConfigFileReader reader(gripper_config_path);
        // communication
        gripper_ip = reader.getValue<std::string>("gripper_ip");
        gripper_state_pub_addr = reader.getValue<std::string>("gripper_state_pub_addr");
        gripper_service_addr = reader.getValue<std::string>("gripper_service_addr");

        // gripper
        gripper_pub_rate_hz = reader.getValue<int>("gripper_pub_rate_hz");
        gripper_command_rcvtimeo_ms = reader.getValue<int>("gripper_command_rcvtimeo_ms");
        gripper_default_close_open_threshold =
            reader.getValue<double>("gripper_default_close_open_threshold");
        gripper_default_speed_slow = reader.getValue<double>("gripper_default_speed_slow");
        gripper_default_speed_fast = reader.getValue<double>("gripper_default_speed_fast");
        gripper_default_force = reader.getValue<double>("gripper_default_force");
    }
};
