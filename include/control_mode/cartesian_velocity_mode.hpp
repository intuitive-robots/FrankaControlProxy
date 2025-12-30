#pragma once
#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/atomic_double_buffer.hpp"


struct CartesianVelocityConfig : public ControllerConfig
{
    // Stiffness
    std::array<double, 7> k_gains;
    // Damping
    std::array<double, 7> d_gains;
    bool limit_rate;
    double cutoff_frequency;

    CartesianVelocityConfig() = default;
    CartesianVelocityConfig(const std::string& controller_config_path)
        : ControllerConfig(controller_config_path) {
    }

    void fromFile(const std::string& controller_config_path) override {
        ConfigFileReader reader(controller_config_path);
        readBaseConfig(reader);
        k_gains = reader.getArray<7>("k_gains", k_gains);
        d_gains = reader.getArray<7>("d_gains", d_gains);
        limit_rate = reader.getValue<bool>("limit_rate", limit_rate);
        cutoff_frequency = reader.getValue<double>("cutoff_frequency", cutoff_frequency);
    }
};



class CartesianVelocityMode : public AbstractControlMode {
public:
    CartesianVelocityMode();
    ~CartesianVelocityMode() override;
    void startControl(AtomicDoubleBuffer<franka::RobotState>& state_buffer) override;
    void stopControl() override;

private:
    AtomicDoubleBuffer<franka::CartesianVelocities> desired_velocities_;
    CartesianVelocityConfig config;
    void initController() override;
    const std::string getModeName() const override;
};
