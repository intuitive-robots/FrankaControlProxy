#include "control_mode/idle_control_mode.hpp"

#include <franka/exception.h>

void IdleControlMode::controlTask()
{
    while (is_running_)
    {
        auto robot_state = robot_->readOnce();
        state_buffer_->write(robot_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    zlc::info("[{}] Control thread ended.", getModeName());
}
