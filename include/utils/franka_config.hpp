#pragma once
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>





class ConfigFileReader {
public:
    ConfigFileReader() = delete;
    virtual ~ConfigFileReader() = default;

    ConfigFileReader(const std::string& arm_config_path) {
        try {
            node = YAML::LoadFile(arm_config_path);
        } catch (const YAML::BadFile& e) {
            zlc::error("Error: Could not open config file: {} ({})", arm_config_path, e.what());
        } catch (const std::exception& e) {
            zlc::error("Error: Failed to parse config: {}", e.what());
        }
    }

    template <typename T>
    T getValue(const std::string& key, const T& fallback) const {
        if (!node[key]) {
            return fallback;
        }
        try {
            return node[key].as<T>();
        } catch (const std::exception& e) {
            zlc::warn("Config parse warning for key {}: {}", key, e.what());
            return fallback;
        }
    }

    template <size_t N>
    std::array<double, N> getArray(const std::string& key, const std::array<double, N>& fallback) const {
        if (!node[key] || !node[key].IsSequence()) {
            return fallback;
        }
        try {
            auto vec = node[key].as<std::vector<double>>();
            if (vec.size() != N) {
                zlc::warn("Config key {} expected {} values, got {}. Using fallback.", key, N, vec.size());
                return fallback;
            }
            std::array<double, N> arr{};
            std::copy(vec.begin(), vec.end(), arr.begin());
            return arr;
        } catch (const std::exception& e) {
            zlc::warn("Config parse warning for key {}: {}", key, e.what());
            return fallback;
        }
    }

private:
    YAML::Node node;

};


struct FrankaArmConfig {
    // communication
    std::string robot_ip;
    std::string state_pub_addr;
    std::string service_addr;

    // arm
    std::array<double, 7> arm_default_state_q;
    std::array<double, 16> arm_default_state_O_T_EE{};
    int arm_state_pub_rate_hz{100};
    int arm_socket_timeout_ms{100};
    int arm_max_message_size{4096};
    // collision behavior thresholds
    std::array<double, 7> arm_col_lower_torque_acc{};
    std::array<double, 7> arm_col_upper_torque_acc{};
    std::array<double, 7> arm_col_lower_torque_nom{};
    std::array<double, 7> arm_col_upper_torque_nom{};
    std::array<double, 6> arm_col_lower_force_acc{};
    std::array<double, 6> arm_col_upper_force_acc{};
    std::array<double, 6> arm_col_lower_force_nom{};
    std::array<double, 6> arm_col_upper_force_nom{};

    void fromFile(const std::string& arm_config_path) {
        ConfigFileReader reader(arm_config_path);
        // defaults for arrays
        const std::array<double, 16> default_O_T_EE{{
            1.0, 0.0, 0.0, 0.3,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.5,
            0.0, 0.0, 0.0, 1.0
        }};

        // communication
        robot_ip = reader.getValue<std::string>("robot_ip", "");
        state_pub_addr = reader.getValue<std::string>("state_pub_addr", "");
        service_addr = reader.getValue<std::string>("service_addr", "");

        // arm
        arm_default_state_q = reader.getArray<7>("arm_default_state_q", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});

        arm_default_state_O_T_EE = reader.getArray<16>("arm_default_state_O_T_EE", default_O_T_EE);
        arm_state_pub_rate_hz = reader.getValue<int>("arm_state_pub_rate_hz", 100);
        arm_socket_timeout_ms = reader.getValue<int>("arm_socket_timeout_ms", 100);
        arm_max_message_size = reader.getValue<int>("arm_max_message_size", 4096);

        // collision behavior (defaults simple non-zero thresholds to avoid 0)
        const std::array<double, 7> d7_acc_low{{30, 30, 30, 30, 30, 30, 30}};
        const std::array<double, 7> d7_acc_up {{45, 45, 45, 45, 45, 45, 45}};
        const std::array<double, 7> d7_nom_low{{25, 25, 25, 25, 25, 25, 25}};
        const std::array<double, 7> d7_nom_up {{35, 35, 35, 35, 35, 35, 35}};
        const std::array<double, 6> d6_acc_low{{10, 10, 10, 10, 10, 10}};
        const std::array<double, 6> d6_acc_up {{15, 15, 15, 15, 15, 15}};
        const std::array<double, 6> d6_nom_low{{8, 8, 8, 8, 8, 8}};
        const std::array<double, 6> d6_nom_up {{12, 12, 12, 12, 12, 12}};

        arm_col_lower_torque_acc = reader.getArray<7>("arm_collision_lower_torque_thresholds_acc", d7_acc_low);
        arm_col_upper_torque_acc = reader.getArray<7>("arm_collision_upper_torque_thresholds_acc", d7_acc_up);
        arm_col_lower_torque_nom = reader.getArray<7>("arm_collision_lower_torque_thresholds_nom", d7_nom_low);
        arm_col_upper_torque_nom = reader.getArray<7>("arm_collision_upper_torque_thresholds_nom", d7_nom_up);
        arm_col_lower_force_acc  = reader.getArray<6>("arm_collision_lower_force_thresholds_acc", d6_acc_low);
        arm_col_upper_force_acc  = reader.getArray<6>("arm_collision_upper_force_thresholds_acc", d6_acc_up);
        arm_col_lower_force_nom  = reader.getArray<6>("arm_collision_lower_force_thresholds_nom", d6_nom_low);
        arm_col_upper_force_nom  = reader.getArray<6>("arm_collision_upper_force_thresholds_nom", d6_nom_up);
        }

};

struct FrankaGripperConfig {
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

    void fromFile(const std::string& gripper_config_path) {
        ConfigFileReader reader(gripper_config_path);
        // communication
        gripper_ip =  reader.getValue<std::string>("gripper_ip", "");
        gripper_state_pub_addr = reader.getValue<std::string>("gripper_state_pub_addr", "");
        gripper_service_addr = reader.getValue<std::string>("gripper_service_addr", "");

        // gripper
        gripper_pub_rate_hz = reader.getValue<int>("gripper_pub_rate_hz", 100);
        gripper_command_rcvtimeo_ms = reader.getValue<int>("gripper_command_rcvtimeo_ms", 500);
        gripper_default_close_open_threshold = reader.getValue<double>("gripper_default_close_open_threshold", 0.01);
        gripper_default_speed_slow = reader.getValue<double>("gripper_default_speed_slow", 0.01);
        gripper_default_speed_fast = reader.getValue<double>("gripper_default_speed_fast", 20.0);
        gripper_default_force = reader.getValue<double>("gripper_default_force", 20.0);
    }

};
