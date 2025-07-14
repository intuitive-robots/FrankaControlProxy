#include <iostream>
#include <fstream>
#include <string>
#include <map>

class RobotConfig {
public:
    std::map<std::string, std::string> config_data;

    // Constructor to initialize and load the configuration from a file
    RobotConfig(const std::string& file_path) {
        loadFromFile(file_path);
    }

    // Function to load configuration from a file
    void loadFromFile(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open config file: " << file_path << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            size_t delimiter_pos = line.find(':');
            if (delimiter_pos != std::string::npos) {
                std::string key = trim(line.substr(0, delimiter_pos));
                std::string value = trim(line.substr(delimiter_pos + 1));

                config_data[key] = value;
            }
        }

        file.close();
    }

    // Function to get a value as a string

    std::string getValue(const std::string& key, const std::string& default_value = "") const {
        auto it = config_data.find(key);
        if (it != config_data.end()) {
            return it->second;
        } else {
        std::cerr << "Warning: Key not found in config: " << key << ", using default: " << default_value << std::endl;
        return default_value;
        }
    }   


    // Function to display the loaded configuration
    void display() const {
        for (const auto& pair : config_data) {
            std::cout << pair.first << ": " << pair.second << std::endl;
        }
    }

private:
    // Function to trim whitespace from a string
    std::string trim(const std::string& str) const {
        size_t first = str.find_first_not_of(" \t");
        size_t last = str.find_last_not_of(" \t");
        if (first == std::string::npos || last == std::string::npos) {
            return "";
        }
        return str.substr(first, last - first + 1);
    }
};
