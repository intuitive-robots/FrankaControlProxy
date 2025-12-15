#pragma once
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "utils/logger.hpp"

struct FrankaArmConfigData {
    // communication
    std::string robot_ip;
    std::string state_pub_addr;
    std::string service_addr;

    // arm
    std::vector<double> arm_default_state_q;
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
};

struct FrankaGripperConfigData {
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
};

class FrankaConfig {
public:
    FrankaConfig() = default;

    // Load separate arm and gripper configs
    void loadFromFiles(const std::string& arm_config_path, const std::string& gripper_config_path) {
        try {
            // Load arm config
            YAML::Node arm_node = YAML::LoadFile(arm_config_path);
            parseArm(arm_node);

            // Load gripper config
            YAML::Node gripper_node = YAML::LoadFile(gripper_config_path);
            parseGripper(gripper_node);
        } catch (const YAML::BadFile& e) {
            LOG_ERROR("Error: Could not open config files: {} or {} ({})", arm_config_path, gripper_config_path, e.what());
        } catch (const std::exception& e) {
            LOG_ERROR("Error: Failed to parse config: {}", e.what());
        }
    }

    const FrankaArmConfigData& armData() const { return arm_data_; }
    const FrankaGripperConfigData& gripperData() const { return gripper_data_; }

private:
    template <typename T>
    static T getOr(const YAML::Node& node, const std::string& key, const T& fallback) {
        if (!node[key]) {
            return fallback;
        }
        try {
            return node[key].as<T>();
        } catch (const std::exception& e) {
            LOG_WARN("Config parse warning for key {}: {}", key, e.what());
            return fallback;
        }
    }

    static std::vector<double> getVecOrEmpty(const YAML::Node& node, const std::string& key) {
        if (!node[key] || !node[key].IsSequence()) {
            return {};
        }
        try {
            return node[key].as<std::vector<double>>();
        } catch (const std::exception& e) {
            LOG_WARN("Config parse warning for key {}: {}", key, e.what());
            return {};
        }
    }

    static std::array<double, 16> getArray16OrDefault(const YAML::Node& node, const std::string& key, const std::array<double, 16>& fallback) {
        if (!node[key] || !node[key].IsSequence()) {
            return fallback;
        }
        try {
            auto vec = node[key].as<std::vector<double>>();
            if (vec.size() != 16) {
                LOG_WARN("Config key {} expected 16 values, got {}. Using fallback.", key, vec.size());
                return fallback;
            }
            std::array<double, 16> arr{};
            std::copy(vec.begin(), vec.end(), arr.begin());
            return arr;
        } catch (const std::exception& e) {
            LOG_WARN("Config parse warning for key {}: {}", key, e.what());
            return fallback;
        }
    }

    template <size_t N>
    static std::array<double, N> getArrayNOrDefault(const YAML::Node& node, const std::string& key, const std::array<double, N>& fallback) {
        if (!node[key] || !node[key].IsSequence()) {
            return fallback;
        }
        try {
            auto vec = node[key].as<std::vector<double>>();
            if (vec.size() != N) {
                LOG_WARN("Config key {} expected {} values, got {}. Using fallback.", key, N, vec.size());
                return fallback;
            }
            std::array<double, N> arr{};
            std::copy(vec.begin(), vec.end(), arr.begin());
            return arr;
        } catch (const std::exception& e) {
            LOG_WARN("Config parse warning for key {}: {}", key, e.what());
            return fallback;
        }
    }

    void parseArm(const YAML::Node& node) {
        // defaults for arrays
        const std::array<double, 16> default_O_T_EE{{
            1.0, 0.0, 0.0, 0.3,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.5,
            0.0, 0.0, 0.0, 1.0
        }};
        const std::vector<double> default_q{{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};

        // communication
        arm_data_.robot_ip = getOr<std::string>(node, "robot_ip", "");
        arm_data_.state_pub_addr = getOr<std::string>(node, "state_pub_addr", "");
        arm_data_.service_addr = getOr<std::string>(node, "service_addr", "");

        // arm
        arm_data_.arm_default_state_q = getVecOrEmpty(node, "arm_default_state_q");
        if (arm_data_.arm_default_state_q.empty()) {
            arm_data_.arm_default_state_q = default_q;
        }
        arm_data_.arm_default_state_O_T_EE = getArray16OrDefault(node, "arm_default_state_O_T_EE", default_O_T_EE);
        arm_data_.arm_state_pub_rate_hz = getOr<int>(node, "arm_state_pub_rate_hz", 100);
        arm_data_.arm_socket_timeout_ms = getOr<int>(node, "arm_socket_timeout_ms", 100);
        arm_data_.arm_max_message_size = getOr<int>(node, "arm_max_message_size", 4096);

        // collision behavior (defaults simple non-zero thresholds to avoid 0)
        const std::array<double, 7> d7_acc_low{{30,30,30,30,30,30,30}};
        const std::array<double, 7> d7_acc_up {{45,45,45,45,45,45,45}};
        const std::array<double, 7> d7_nom_low{{25,25,25,25,25,25,25}};
        const std::array<double, 7> d7_nom_up {{35,35,35,35,35,35,35}};
        const std::array<double, 6> d6_acc_low{{10,10,10,10,10,10}};
        const std::array<double, 6> d6_acc_up {{15,15,15,15,15,15}};
        const std::array<double, 6> d6_nom_low{{8,8,8,8,8,8}};
        const std::array<double, 6> d6_nom_up {{12,12,12,12,12,12}};

        arm_data_.arm_col_lower_torque_acc = getArrayNOrDefault<7>(node, "arm_collision_lower_torque_thresholds_acc", d7_acc_low);
        arm_data_.arm_col_upper_torque_acc = getArrayNOrDefault<7>(node, "arm_collision_upper_torque_thresholds_acc", d7_acc_up);
        arm_data_.arm_col_lower_torque_nom = getArrayNOrDefault<7>(node, "arm_collision_lower_torque_thresholds_nom", d7_nom_low);
        arm_data_.arm_col_upper_torque_nom = getArrayNOrDefault<7>(node, "arm_collision_upper_torque_thresholds_nom", d7_nom_up);
        arm_data_.arm_col_lower_force_acc  = getArrayNOrDefault<6>(node, "arm_collision_lower_force_thresholds_acc", d6_acc_low);
        arm_data_.arm_col_upper_force_acc  = getArrayNOrDefault<6>(node, "arm_collision_upper_force_thresholds_acc", d6_acc_up);
        arm_data_.arm_col_lower_force_nom  = getArrayNOrDefault<6>(node, "arm_collision_lower_force_thresholds_nom", d6_nom_low);
        arm_data_.arm_col_upper_force_nom  = getArrayNOrDefault<6>(node, "arm_collision_upper_force_thresholds_nom", d6_nom_up);
    }

    void parseGripper(const YAML::Node& node) {
        // communication
        gripper_data_.gripper_ip = getOr<std::string>(node, "gripper_ip", "");
        gripper_data_.gripper_state_pub_addr = getOr<std::string>(node, "gripper_state_pub_addr", "");
        gripper_data_.gripper_service_addr = getOr<std::string>(node, "gripper_service_addr", "");

        // gripper
        gripper_data_.gripper_pub_rate_hz = getOr<int>(node, "gripper_pub_rate_hz", 100);
        gripper_data_.gripper_command_rcvtimeo_ms = getOr<int>(node, "gripper_command_rcvtimeo_ms", 500);
        gripper_data_.gripper_default_close_open_threshold = getOr<double>(node, "gripper_default_close_open_threshold", 0.01);
        gripper_data_.gripper_default_speed_slow = getOr<double>(node, "gripper_default_speed_slow", 0.01);
        gripper_data_.gripper_default_speed_fast = getOr<double>(node, "gripper_default_speed_fast", 20.0);
        gripper_data_.gripper_default_force = getOr<double>(node, "gripper_default_force", 20.0);
    }

private:
    FrankaArmConfigData arm_data_;
    FrankaGripperConfigData gripper_data_;
};
