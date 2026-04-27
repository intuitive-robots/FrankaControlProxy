#pragma once

#include "control_mode/abstract_control_mode.hpp"

class HumanControlMode : public AbstractControlMode
{
  public:
    HumanControlMode()
    {
        controller_name = "HumanControl";
    };
    ~HumanControlMode() override = default;

  private:
    franka::Torques controlLoop(const franka::RobotState& robot_state,
                                franka::Duration duration) override;
};
