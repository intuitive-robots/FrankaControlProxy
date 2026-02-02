#pragma once

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>

#include "utils/atomic_double_buffer.hpp"

class JointPositionMotionGenerator 
{
  public:
    /**
     * Creates a new JointPositionMotionGenerator instance for a target q.
     *
     * @param[in] speed_factor General speed factor in range [0, 1].
     * @param[in] q_goal Target joint positions.
     * @param[in] state_buffer Buffer to write robot state updates during motion.
     * @param[in] tolerance If max joint error is below this threshold, motion finishes immediately (default: 1e-3 rad).
     */
    JointPositionMotionGenerator(double speed_factor, const std::array<double, 7>& q_goal,
                                 AtomicDoubleBuffer<franka::RobotState>& state_buffer,
                                 double tolerance = 1e-3);

      /**
       * Sends joint position calculations
       *
       * @param[in] robot_state Current state of the robot.
       * @param[in] period Duration of execution.
       *
       * @return Joint positions for use inside a control loop.
       */
      franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

    private:
      using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
      using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

      bool calculateDesiredValues(double time, Vector7d* delta_q_d) const;
      void calculateSynchronizedValues();

      static constexpr double kDeltaQMotionFinished = 1e-6;
      Vector7d q_goal_;

      Vector7d q_start_;
      Vector7d delta_q_;

      Vector7d dq_max_sync_;
      Vector7d t_1_sync_;
      Vector7d t_2_sync_;
      Vector7d t_f_sync_;
      Vector7d q_1_;

      double time_ = 0.0;

      Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
      Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
      Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();

      AtomicDoubleBuffer<franka::RobotState>* state_buffer_;
      double tolerance_;
};
