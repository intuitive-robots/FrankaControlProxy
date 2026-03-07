#pragma once

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#if NO_ROBOT_TESTING
#include "mujoco_sim/mujoco_robot.hpp"
using FrankaPanda = MujocoRobot;
#else
using FrankaPanda = franka::Robot;
#endif