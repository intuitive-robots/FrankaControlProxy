#pragma once
#include "control_mode/abstract_control_mode.hpp"
#include <array>

// Hand-guiding controller with joint-space spring + damping (no gravity compensation).
// Intended for the "leader" robot so a human can move it by hand.

class GravityCompControl : public AbstractControlMode
{
public:
    explicit GravityCompControl(const SafetyLimitConfig& safety_config, const std::string& robot_name)
        : AbstractControlMode(safety_config, robot_name)
    {
        controller_name = "GravityComp";
    }
    ~GravityCompControl() override = default;

    // Reset internal state. Call this right before starting control.
    void reset();
    
private:
    void initController(FrankaPanda& robot, PandaPinocchioModel& pinocchio_model,
                        AtomicDoubleBuffer<franka::RobotState>& state_buffer) override;
    
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;

    bool hold_initialized_{false};
    JointPosition hold_q_{JointPosition::Zero()};
};
