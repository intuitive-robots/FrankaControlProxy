#pragma once

#include "control_mode/abstract_control_mode.hpp"

class IdleControlMode : public AbstractControlMode
{
  public:
    IdleControlMode(const SafetyLimitConfig& safety_config) : AbstractControlMode(safety_config)
    {
        controller_name = "IdleControlMode";
    };
    ~IdleControlMode() override = default;

  private:
    void initController(FrankaPanda& robot, PandaPinocchioModel& model,
                        AtomicDoubleBuffer<franka::RobotState>& state_buffer) override;
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
};
