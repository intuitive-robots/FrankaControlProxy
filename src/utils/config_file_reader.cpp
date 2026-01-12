#include "utils/config_file_reader.hpp"

ConfigFileReader::ConfigFileReader(const std::string& arm_config_path)
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
