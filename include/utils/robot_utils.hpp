#pragma once

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <fstream>
#include <memory>
#include <string>
#include <zerolancom/zerolancom.hpp>

#if NO_ROBOT_TESTING
#include "mujoco_sim/mujoco_robot.hpp"
using FrankaPanda = MujocoRobot;
#else
#include <franka/control_tools.h>
using FrankaPanda = franka::Robot;
#endif

// franka::hasRealtimeKernel() (libfranka 0.9.2) returns an uninitialized bool when /sys/kernel/realtime
// does not exist, i.e. on every non-realtime kernel, so it can claim realtime at random.
inline bool hasRealtimeKernel()
{
    std::ifstream realtime("/sys/kernel/realtime");
    int is_realtime = 0;
    realtime >> is_realtime;
    return is_realtime == 1;
}

// libfranka refuses to control the robot on a non-realtime kernel (RealtimeConfig::kEnforce).
// There we fall back to kIgnore, which also skips raising the control thread to realtime priority,
// so setControlThreadPriority() has to be called before every robot.control().
inline std::unique_ptr<FrankaPanda> makeFrankaPanda(const std::string& robot_ip)
{
#if NO_ROBOT_TESTING
    return std::make_unique<FrankaPanda>(robot_ip);
#else
    if (hasRealtimeKernel())
    {
        return std::make_unique<FrankaPanda>(robot_ip, franka::RealtimeConfig::kEnforce);
    }
    zlc::warn("Running kernel does not have realtime capabilities, ignoring libfranka's realtime check.");
    return std::make_unique<FrankaPanda>(robot_ip, franka::RealtimeConfig::kIgnore);
#endif
}

// Same as libfranka does for kEnforce: SCHED_FIFO with the highest priority for the calling thread.
inline void setControlThreadPriority()
{
#if !NO_ROBOT_TESTING
    if (hasRealtimeKernel())
    {
        return;  // kEnforce: libfranka sets the priority itself
    }
    std::string error_message;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&error_message))
    {
        throw franka::RealtimeException(error_message);
    }
#endif
}