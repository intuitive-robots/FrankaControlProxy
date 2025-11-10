#ifndef CARTESIAN_POSITION_MODE_HPP
#define CARTESIAN_POSITION_MODE_HPP

#include "abstract_control_mode.hpp"
#include <franka/robot_state.h>
#include <functional>
#include "utils/atomic_double_buffer.hpp"

/**
 * @brief Cartesian position control mode.
 *
 * Controls the end-effector position and orientation in Cartesian space
 * using franka::CartesianPose commands.
 */
class CartesianPositionMode : public AbstractControlMode {
public:
    CartesianPositionMode();
    ~CartesianPositionMode() override;

    void start();
    void stop() override;
    int getModeID() const override;

private:
    AtomicDoubleBuffer<franka::CartesianPose> desired_pose_;
};

#endif // CARTESIAN_POSITION_MODE_HPP
