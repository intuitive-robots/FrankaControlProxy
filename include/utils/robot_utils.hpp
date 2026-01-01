#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include "debugger/fake_franka.hpp"

#if NO_ROBOT_TESTING
using FrankaPanda = FakeFrankaRobot;
using FrankaModel = FakeFrankaModel;
#else
using FrankaPanda = franka::Robot;
using FrankaModel = franka::Model;
#endif