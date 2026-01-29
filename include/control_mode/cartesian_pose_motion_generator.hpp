#pragma once

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>

class CartesianPoseMotionGenerator 
{
  public:
    /**
     * Creates a new CartesianPoseMotionGenerator instance for a target pose.
     *
     * @param[in] speed_factor General speed factor in range [0, 1].
     * @param[in] goal_position Target position [x, y, z] in meters.
     * @param[in] goal_orientation Target orientation as quaternion.
     * @param[in] dx_max Maximum translational velocity in m/s (default: 0.3).
     * @param[in] ddx_max_start Maximum translational acceleration at start in m/s² (default: 1.0).
     * @param[in] ddx_max_goal Maximum translational acceleration at goal in m/s² (default: 1.0).
     * @param[in] omega_max Maximum rotational velocity in rad/s (default: 0.5).
     * @param[in] domega_max_start Maximum rotational acceleration at start in rad/s² (default: 1.0).
     * @param[in] domega_max_goal Maximum rotational acceleration at goal in rad/s² (default: 1.0).
     */
    CartesianPoseMotionGenerator(double speed_factor,
                                  const Eigen::Vector3d& goal_position,
                                  const Eigen::Quaterniond& goal_orientation,
                                  double dx_max = 0.3,
                                  double ddx_max_start = 1.0,
                                  double ddx_max_goal = 1.0,
                                  double omega_max = 0.5,
                                  double domega_max_start = 1.0,
                                  double domega_max_goal = 1.0);

    /**
     * Sends Cartesian pose calculations
     *
     * @param[in] robot_state Current state of the robot.
     * @param[in] period Duration of execution.
     *
     * @return Cartesian pose for use inside a control loop.
     */
    franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:
    using Vector3d = Eigen::Vector3d;

    bool calculateDesiredValues(double time, Vector3d* delta_pos_d, double* alpha) const;
    void calculateSynchronizedValues();
    std::array<double, 16> poseToMatrix(const Vector3d& position, const Eigen::Quaterniond& orientation) const;

    static constexpr double kDeltaPosMotionFinished = 1e-6;  // meters
    static constexpr double kDeltaRotMotionFinished = 1e-6;  // radians

    // Goal pose
    Vector3d pos_goal_;
    Eigen::Quaterniond q_goal_;

    // Start pose
    Vector3d pos_start_;
    Eigen::Quaterniond q_start_;

    // Position delta
    Vector3d delta_pos_;

    // Rotation angle (total angle to rotate)
    double delta_rot_;

    // Position timing (synchronized for x, y, z)
    Vector3d dx_max_sync_;
    Vector3d t_1_sync_pos_;
    Vector3d t_2_sync_pos_;
    Vector3d t_f_sync_pos_;
    Vector3d pos_1_;  // position at end of acceleration phase

    // Rotation timing
    double omega_max_sync_;
    double t_1_sync_rot_;
    double t_2_sync_rot_;
    double t_f_sync_rot_;
    double rot_1_;  // rotation at end of acceleration phase

    // Global synchronized finish time
    double t_f_sync_;

    double time_ = 0.0;

    // Translational velocity/acceleration limits
    Vector3d dx_max_;
    Vector3d ddx_max_start_;
    Vector3d ddx_max_goal_;

    // Rotational velocity/acceleration limits
    double omega_max_;
    double domega_max_start_;
    double domega_max_goal_;
};
