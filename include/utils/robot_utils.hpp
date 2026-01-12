#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "mujoco_sim/mujoco_robot.hpp"

#if NO_ROBOT_TESTING
using FrankaPanda = MujocoRobot;
#else
using FrankaPanda = franka::Robot;
#endif