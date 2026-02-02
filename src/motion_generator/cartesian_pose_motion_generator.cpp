#include "motion_generator/cartesian_pose_motion_generator.hpp"

#include <algorithm>
#include <cmath>

CartesianPoseMotionGenerator::CartesianPoseMotionGenerator(
    double speed_factor,
    const Eigen::Vector3d& goal_position,
    const Eigen::Quaterniond& goal_orientation,
    AtomicDoubleBuffer<franka::RobotState>& state_buffer,
    double tolerance,
    double dx_max,
    double ddx_max_start,
    double ddx_max_goal,
    double omega_max,
    double domega_max_start,
    double domega_max_goal)
    : pos_goal_(goal_position),
      q_goal_(goal_orientation.normalized()),
      dx_max_(Vector3d::Constant(dx_max * speed_factor)),
      ddx_max_start_(Vector3d::Constant(ddx_max_start * speed_factor)),
      ddx_max_goal_(Vector3d::Constant(ddx_max_goal * speed_factor)),
      omega_max_(omega_max * speed_factor),
      domega_max_start_(domega_max_start * speed_factor),
      domega_max_goal_(domega_max_goal * speed_factor),
      state_buffer_(&state_buffer),
      tolerance_(tolerance) {
    dx_max_sync_.setZero();
    pos_start_.setZero();
    delta_pos_.setZero();
    t_1_sync_pos_.setZero();
    t_2_sync_pos_.setZero();
    t_f_sync_pos_.setZero();
    pos_1_.setZero();

    omega_max_sync_ = 0.0;
    t_1_sync_rot_ = 0.0;
    t_2_sync_rot_ = 0.0;
    t_f_sync_rot_ = 0.0;
    rot_1_ = 0.0;
    delta_rot_ = 0.0;
    t_f_sync_ = 0.0;
}

franka::CartesianPose CartesianPoseMotionGenerator::operator()(
    const franka::RobotState& robot_state,
    franka::Duration period) {
    state_buffer_->write(robot_state);
    time_ += period.toSec();

    if (time_ == 0.0) {
        // Extract start position from O_T_EE (column-major 4x4 matrix)
        pos_start_ = Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);

        // Extract start orientation from O_T_EE rotation matrix
        Eigen::Matrix3d rot_matrix;
        rot_matrix << robot_state.O_T_EE[0], robot_state.O_T_EE[4], robot_state.O_T_EE[8],
                      robot_state.O_T_EE[1], robot_state.O_T_EE[5], robot_state.O_T_EE[9],
                      robot_state.O_T_EE[2], robot_state.O_T_EE[6], robot_state.O_T_EE[10];
        q_start_ = Eigen::Quaterniond(rot_matrix).normalized();

        // Ensure shortest path for quaternion SLERP
        if (q_start_.dot(q_goal_) < 0.0) {
            q_goal_.coeffs() *= -1.0;
        }

        delta_pos_ = pos_goal_ - pos_start_;

        // Calculate total rotation angle
        Eigen::Quaterniond q_diff = q_start_.inverse() * q_goal_;
        delta_rot_ = 2.0 * std::acos(std::clamp(std::abs(q_diff.w()), 0.0, 1.0));

        // Check if already within tolerance - skip motion if so
        if (delta_pos_.norm() < tolerance_ && delta_rot_ < tolerance_) {
            franka::CartesianPose output(robot_state.O_T_EE);
            output.motion_finished = true;
            return output;
        }

        calculateSynchronizedValues();
    }

    Vector3d delta_pos_d;
    double alpha;
    bool motion_finished = calculateDesiredValues(time_, &delta_pos_d, &alpha);

    // Interpolate position
    Vector3d current_pos = pos_start_ + delta_pos_d;

    // Interpolate orientation using SLERP
    Eigen::Quaterniond current_quat = q_start_.slerp(alpha, q_goal_);

    // Convert to 4x4 matrix
    std::array<double, 16> pose_matrix = poseToMatrix(current_pos, current_quat);

    franka::CartesianPose output(pose_matrix);
    output.motion_finished = motion_finished;
    return output;
}

bool CartesianPoseMotionGenerator::calculateDesiredValues(
    double time,
    Vector3d* delta_pos_d,
    double* alpha) const {
    
    // Position interpolation (same S-curve as joint version, but for 3 axes)
    Eigen::Vector3i sign_delta_pos;
    for (int i = 0; i < 3; i++) {
        sign_delta_pos[i] = (delta_pos_[i] >= 0) ? 1 : -1;
    }
    Vector3d t_d_pos = t_2_sync_pos_ - t_1_sync_pos_;
    Vector3d delta_t_2_sync_pos = t_f_sync_pos_ - t_2_sync_pos_;
    
    std::array<bool, 3> pos_motion_finished{};

    for (int i = 0; i < 3; i++) {
        if (std::abs(delta_pos_[i]) < kDeltaPosMotionFinished) {
            (*delta_pos_d)[i] = 0;
            pos_motion_finished[i] = true;
        } else {
            if (time < t_1_sync_pos_[i]) {
                (*delta_pos_d)[i] = -1.0 / std::pow(t_1_sync_pos_[i], 3.0) * dx_max_sync_[i] * sign_delta_pos[i] *
                                (0.5 * time - t_1_sync_pos_[i]) * std::pow(time, 3.0);
            } else if (time >= t_1_sync_pos_[i] && time < t_2_sync_pos_[i]) {
                (*delta_pos_d)[i] = pos_1_[i] + (time - t_1_sync_pos_[i]) * dx_max_sync_[i] * sign_delta_pos[i];
            } else if (time >= t_2_sync_pos_[i] && time < t_f_sync_pos_[i]) {
                (*delta_pos_d)[i] =
                    delta_pos_[i] +
                    0.5 *
                        (1.0 / std::pow(delta_t_2_sync_pos[i], 3.0) *
                            (time - t_1_sync_pos_[i] - 2.0 * delta_t_2_sync_pos[i] - t_d_pos[i]) *
                            std::pow((time - t_1_sync_pos_[i] - t_d_pos[i]), 3.0) +
                        (2.0 * time - 2.0 * t_1_sync_pos_[i] - delta_t_2_sync_pos[i] - 2.0 * t_d_pos[i])) *
                        dx_max_sync_[i] * sign_delta_pos[i];
            } else {
                (*delta_pos_d)[i] = delta_pos_[i];
                pos_motion_finished[i] = true;
            }
        }
    }

    // Rotation interpolation (S-curve for alpha parameter)
    bool rot_motion_finished = false;
    if (delta_rot_ < kDeltaRotMotionFinished) {
        *alpha = 1.0;
        rot_motion_finished = true;
    } else {
        int sign_delta_rot = 1;  // Always positive since delta_rot_ is absolute angle
        double t_d_rot = t_2_sync_rot_ - t_1_sync_rot_;
        double delta_t_2_sync_rot = t_f_sync_rot_ - t_2_sync_rot_;
        double delta_rot_d;

        if (time < t_1_sync_rot_) {
            delta_rot_d = -1.0 / std::pow(t_1_sync_rot_, 3.0) * omega_max_sync_ * sign_delta_rot *
                          (0.5 * time - t_1_sync_rot_) * std::pow(time, 3.0);
        } else if (time >= t_1_sync_rot_ && time < t_2_sync_rot_) {
            delta_rot_d = rot_1_ + (time - t_1_sync_rot_) * omega_max_sync_ * sign_delta_rot;
        } else if (time >= t_2_sync_rot_ && time < t_f_sync_rot_) {
            delta_rot_d =
                delta_rot_ +
                0.5 *
                    (1.0 / std::pow(delta_t_2_sync_rot, 3.0) *
                        (time - t_1_sync_rot_ - 2.0 * delta_t_2_sync_rot - t_d_rot) *
                        std::pow((time - t_1_sync_rot_ - t_d_rot), 3.0) +
                    (2.0 * time - 2.0 * t_1_sync_rot_ - delta_t_2_sync_rot - 2.0 * t_d_rot)) *
                    omega_max_sync_ * sign_delta_rot;
        } else {
            delta_rot_d = delta_rot_;
            rot_motion_finished = true;
        }

        // Convert to alpha [0, 1] for SLERP
        *alpha = std::clamp(delta_rot_d / delta_rot_, 0.0, 1.0);
    }

    return std::all_of(pos_motion_finished.cbegin(), pos_motion_finished.cend(),
                       [](bool is_finished) { return is_finished; }) && rot_motion_finished;
}

void CartesianPoseMotionGenerator::calculateSynchronizedValues() {
    // === Position synchronization (for 3 axes) ===
    Vector3d dx_max_reach(dx_max_);
    Vector3d t_f_pos = Vector3d::Zero();
    Vector3d delta_t_2_pos = Vector3d::Zero();
    Vector3d t_1_pos = Vector3d::Zero();
    Vector3d delta_t_2_sync_pos = Vector3d::Zero();
    Eigen::Vector3i sign_delta_pos;
    for (int i = 0; i < 3; i++) {
        sign_delta_pos[i] = (delta_pos_[i] >= 0) ? 1 : -1;
    }

    for (int i = 0; i < 3; i++) {
        if (std::abs(delta_pos_[i]) > kDeltaPosMotionFinished) {
            if (std::abs(delta_pos_[i]) < (3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_start_[i]) +
                                        3.0 / 4.0 * (std::pow(dx_max_[i], 2.0) / ddx_max_goal_[i]))) {
                dx_max_reach[i] = std::sqrt(4.0 / 3.0 * std::abs(delta_pos_[i]) *
                                            (ddx_max_start_[i] * ddx_max_goal_[i]) /
                                            (ddx_max_start_[i] + ddx_max_goal_[i]));
            }
            t_1_pos[i] = 1.5 * dx_max_reach[i] / ddx_max_start_[i];
            delta_t_2_pos[i] = 1.5 * dx_max_reach[i] / ddx_max_goal_[i];
            t_f_pos[i] = t_1_pos[i] / 2.0 + delta_t_2_pos[i] / 2.0 + std::abs(delta_pos_[i]) / dx_max_reach[i];
        }
    }

    double max_t_f_pos = t_f_pos.maxCoeff();

    // === Rotation synchronization ===
    double omega_max_reach = omega_max_;
    double t_f_rot = 0.0;
    double delta_t_2_rot = 0.0;
    double t_1_rot = 0.0;
    double delta_t_2_sync_rot = 0.0;

    if (delta_rot_ > kDeltaRotMotionFinished) {
        if (delta_rot_ < (3.0 / 4.0 * (std::pow(omega_max_, 2.0) / domega_max_start_) +
                         3.0 / 4.0 * (std::pow(omega_max_, 2.0) / domega_max_goal_))) {
            omega_max_reach = std::sqrt(4.0 / 3.0 * delta_rot_ *
                                        (domega_max_start_ * domega_max_goal_) /
                                        (domega_max_start_ + domega_max_goal_));
        }
        t_1_rot = 1.5 * omega_max_reach / domega_max_start_;
        delta_t_2_rot = 1.5 * omega_max_reach / domega_max_goal_;
        t_f_rot = t_1_rot / 2.0 + delta_t_2_rot / 2.0 + delta_rot_ / omega_max_reach;
    }

    // === Synchronize position and rotation to same finish time ===
    t_f_sync_ = std::max(max_t_f_pos, t_f_rot);

    // Re-compute position timing for synchronized finish
    for (int i = 0; i < 3; i++) {
        if (std::abs(delta_pos_[i]) > kDeltaPosMotionFinished) {
            double a = 1.5 / 2.0 * (ddx_max_goal_[i] + ddx_max_start_[i]);
            double b = -1.0 * t_f_sync_ * ddx_max_goal_[i] * ddx_max_start_[i];
            double c = std::abs(delta_pos_[i]) * ddx_max_goal_[i] * ddx_max_start_[i];
            double delta = b * b - 4.0 * a * c;
            if (delta < 0.0) {
                delta = 0.0;
            }
            dx_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
            t_1_sync_pos_[i] = 1.5 * dx_max_sync_[i] / ddx_max_start_[i];
            delta_t_2_sync_pos[i] = 1.5 * dx_max_sync_[i] / ddx_max_goal_[i];
            t_f_sync_pos_[i] = t_1_sync_pos_[i] / 2.0 + delta_t_2_sync_pos[i] / 2.0 + 
                               std::abs(delta_pos_[i] / dx_max_sync_[i]);
            t_2_sync_pos_[i] = t_f_sync_pos_[i] - delta_t_2_sync_pos[i];
            pos_1_[i] = dx_max_sync_[i] * sign_delta_pos[i] * (0.5 * t_1_sync_pos_[i]);
        }
    }

    // Re-compute rotation timing for synchronized finish
    if (delta_rot_ > kDeltaRotMotionFinished) {
        double a = 1.5 / 2.0 * (domega_max_goal_ + domega_max_start_);
        double b = -1.0 * t_f_sync_ * domega_max_goal_ * domega_max_start_;
        double c = delta_rot_ * domega_max_goal_ * domega_max_start_;
        double delta = b * b - 4.0 * a * c;
        if (delta < 0.0) {
            delta = 0.0;
        }
        omega_max_sync_ = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_rot_ = 1.5 * omega_max_sync_ / domega_max_start_;
        delta_t_2_sync_rot = 1.5 * omega_max_sync_ / domega_max_goal_;
        t_f_sync_rot_ = t_1_sync_rot_ / 2.0 + delta_t_2_sync_rot / 2.0 + delta_rot_ / omega_max_sync_;
        t_2_sync_rot_ = t_f_sync_rot_ - delta_t_2_sync_rot;
        rot_1_ = omega_max_sync_ * (0.5 * t_1_sync_rot_);
    }
}

std::array<double, 16> CartesianPoseMotionGenerator::poseToMatrix(
    const Vector3d& position,
    const Eigen::Quaterniond& orientation) const {
    
    Eigen::Matrix3d rot_matrix = orientation.normalized().toRotationMatrix();
    
    // Column-major 4x4 homogeneous transformation matrix
    std::array<double, 16> matrix{};
    
    // Rotation part (columns 0-2)
    matrix[0] = rot_matrix(0, 0);
    matrix[1] = rot_matrix(1, 0);
    matrix[2] = rot_matrix(2, 0);
    matrix[3] = 0.0;
    
    matrix[4] = rot_matrix(0, 1);
    matrix[5] = rot_matrix(1, 1);
    matrix[6] = rot_matrix(2, 1);
    matrix[7] = 0.0;
    
    matrix[8] = rot_matrix(0, 2);
    matrix[9] = rot_matrix(1, 2);
    matrix[10] = rot_matrix(2, 2);
    matrix[11] = 0.0;
    
    // Translation part (column 3)
    matrix[12] = position[0];
    matrix[13] = position[1];
    matrix[14] = position[2];
    matrix[15] = 1.0;
    
    return matrix;
}
