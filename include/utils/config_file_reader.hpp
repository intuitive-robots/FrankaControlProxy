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

    ConfigFileReader(const std::string& arm_config_path);

    YAML::Node getSubNode(const std::string& key) const
    {
        return node[key];
    }

    template <typename T>
    T getValue(const std::string& key) const
    {
        if (!node[key])
        {
            throw ConfigValueNotFound(key);
        }
        return _getValue<T>(key);
    }

    template <typename T>
    T getValue(const std::string& key, const T& default_value) const
    {
        if (!node[key])
        {
            return default_value;
        }
        return _getValue<T>(key);
    }

    template <typename T, size_t N>
    std::array<T, N> getArray(const std::string& key) const
    {
        if (!node[key] || !node[key].IsSequence())
        {
            throw ConfigValueNotFound(key);
        }
        return _getArray<T, N>(key);
    }

    template <typename T, size_t N>
    std::array<T, N> getArray(const std::string& key, const std::array<T, N>& default_value) const
    {
        if (!node[key] || !node[key].IsSequence())
        {
            return default_value;
        }
        return _getArray<T, N>(key);
    }

  private:
    YAML::Node node;

    template <typename T>
    T _getValue(const std::string& key) const
    {
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
    std::array<T, N> _getArray(const std::string& key) const
    {
        try
        {
            auto vec = node[key].as<std::vector<T>>();
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
};
