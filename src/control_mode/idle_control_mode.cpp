#include "control_mode/idle_control_mode.hpp"

#include <franka/exception.h>

void IdleControlMode::initController(FrankaPanda& robot, PandaPinocchioModel& model,
                                     AtomicDoubleBuffer<franka::RobotState>& state_buffer)
{
    AbstractControlMode::initController(robot, model, state_buffer);
    zlc::info("[IdleControlMode] Initialized.");
}

franka::Torques IdleControlMode::controlLoop(const franka::RobotState& robot_state,
                                             franka::Duration /*duration*/)
{
    if (state_buffer_)
    {
        state_buffer_->write(robot_state);
    }
    return franka::Torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}
