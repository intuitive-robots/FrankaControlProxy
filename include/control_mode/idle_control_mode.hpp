#pragma once

#include "control_mode/abstract_control_mode.hpp"

class IdleControlMode : public AbstractControlMode
{
  public:
    IdleControlMode(const SafetyLimitConfig& safety_config, const std::string& robot_name) : AbstractControlMode(safety_config, robot_name)
    {
        controller_name = "IdleControlMode";
    };
    ~IdleControlMode() override = default;

  private:
    void initController(FrankaPanda& robot, PandaPinocchioModel& model,
                        AtomicDoubleBuffer<franka::RobotState>& state_buffer) override;
    void controlTask() override;
};
