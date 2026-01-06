#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include "mujoco_sim/mujoco_robot.hpp"

#if NO_ROBOT_TESTING
using FrankaPanda = MujocoRobot;
using FrankaModel = MujocoModel;
#else
using FrankaPanda = franka::Robot;
using FrankaModel = franka::Model;
#endif