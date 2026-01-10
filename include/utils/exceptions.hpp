#pragma once

#include <stdexcept>
#include <string>

class FrankaControlProxyException : public std::runtime_error
{
  public:
    explicit FrankaControlProxyException(const std::string& message) : std::runtime_error(message)
    {
    }
};

class ConfigValueNotFound : public FrankaControlProxyException
{
  public:
    explicit ConfigValueNotFound(const std::string& key)
        : FrankaControlProxyException("Config key " + key + " not found")
    {
    }
};
