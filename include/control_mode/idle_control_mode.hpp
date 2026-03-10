#pragma once

#include "control_mode/abstract_control_mode.hpp"

class IdleControlMode : public AbstractControlMode
{
  public:
    IdleControlMode()
    {
        controller_name = "IdleControlMode";
    };
    ~IdleControlMode() override = default;

  private:
    void controlTask() override;
};
