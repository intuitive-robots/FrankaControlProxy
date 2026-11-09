#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ruckig/ruckig.hpp>

#include "utils/Pose.h"
#include "utils/config_file_reader.hpp"

// Desired pose and feed-forward twist (base frame: v [m/s], omega [rad/s]) of one command,
// stored together so the control loop never mixes pose and twist of different commands.
struct CartesianTarget
{
    transform::Pose pose{transform::Pose::Identity()};
    Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};
    std::chrono::steady_clock::time_point stamp{};
};

struct CartesianFeedforwardConfig
{
    // Scale of the commanded twist used as target velocity and for extrapolation (0 = off)
    double velocity_feedforward{1.0};
    // Shift the target pose along the commanded twist between commands
    bool extrapolate_pose{true};
    // Feed-forward and extrapolation stop when the last command is older than this [s]
    double feedforward_timeout{0.1};
    // Reference trajectory (see CartesianReferenceGenerator):
    // gain [1/s] of the loop that pulls the reference onto the target
    double reference_position_gain{30.0};
    // fraction of max_acceleration the loop plans to brake with (rest is jerk margin)
    double reference_braking{0.5};
    // low-pass time constant [s] of the commanded twist
    double reference_velocity_filter_time{0.02};
    // Limits of the reference per base-frame axis [x, y, z (m), rx, ry, rz (rad)].
    // Target motions beyond them lag instead of reaching the robot.
    std::array<double, 6> max_velocity{};
    std::array<double, 6> max_acceleration{};
    std::array<double, 6> max_jerk{};

    void read(const ConfigFileReader& reader)
    {
        velocity_feedforward = reader.getValue<double>("velocity_feedforward", 1.0);
        extrapolate_pose = reader.getValue<bool>("extrapolate_pose", true);
        feedforward_timeout = reader.getValue<double>("feedforward_timeout", 0.1);
        reference_position_gain = reader.getValue<double>("reference_position_gain", 30.0);
        reference_braking = reader.getValue<double>("reference_braking", 0.5);
        reference_velocity_filter_time = reader.getValue<double>("reference_velocity_filter_time", 0.02);
        // required: every preset has to state its limits
        max_velocity = reader.getArray<double, 6>("max_velocity");
        max_acceleration = reader.getArray<double, 6>("max_acceleration");
        max_jerk = reader.getArray<double, 6>("max_jerk");
    }
};

// Rotation vector (axis * angle, angle in [0, pi]) of q, i.e. the SO(3) logarithm
inline Eigen::Vector3d rotationVector(Eigen::Quaterniond q)
{
    if (q.w() < 0.0)
    {
        q.coeffs() *= -1.0;  // shortest path
    }
    const double s = q.vec().norm();
    if (s < 1e-12)
    {
        return 2.0 * q.vec();  // small-angle limit
    }
    return (2.0 * std::atan2(s, q.w()) / s) * q.vec();
}

// Rotation of rotation vector v, i.e. the SO(3) exponential
inline Eigen::Quaterniond rotationFromVector(const Eigen::Vector3d& v)
{
    const double angle = v.norm();
    if (angle < 1e-12)
    {
        Eigen::Quaterniond q(1.0, 0.5 * v.x(), 0.5 * v.y(), 0.5 * v.z());
        return q.normalized();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

// Target pose and twist of the latest command for the current control tick. Without a commanded
// twist this reduces to the plain pose target with zero target twist.
inline void cartesianTarget(const CartesianTarget& target, const CartesianFeedforwardConfig& config,
                            Eigen::Vector3d& target_pos, Eigen::Quaterniond& target_quat,
                            Eigen::Matrix<double, 6, 1>& target_twist)
{
    const double cmd_age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - target.stamp).count();
    const Eigen::Matrix<double, 6, 1> twist = config.velocity_feedforward * target.twist;
    target_twist = cmd_age < config.feedforward_timeout ? twist : Eigen::Matrix<double, 6, 1>::Zero();
    target_pos = target.pose.translation();
    target_quat = target.pose.quaternion();
    if (config.extrapolate_pose)
    {
        // Capped at the timeout so the target stops (and does not jump back) when commands stop
        const Eigen::Matrix<double, 6, 1> shift = twist * std::min(cmd_age, config.feedforward_timeout);
        target_pos += shift.head<3>();
        target_quat = (rotationFromVector(shift.tail<3>()) * target_quat).normalized();
    }
}

// Reference pose for the Cartesian impedance controller, so it never sees the raw (stepping,
// noisy, possibly jumping) target. Same structure as JointReferenceGenerator (see there for why
// Ruckig's position interface is not used), with orientation handled on SO(3):
// - desired twist = filtered commanded twist + a loop on the target error. Translation uses the
//   position error, rotation the rotation vector log(q_target * q_ref^-1) in the base frame, so
//   angles never accumulate or wrap. Each loop is linear for small errors and limited to
//   sqrt(2 * braking * a_max * |error|) for large ones (stops at the target without overshoot),
//   and acts along the error vector so a large step still moves on a straight line / one axis.
// - Ruckig (velocity interface, 6 DoF) reaches that twist within max_velocity/acceleration/jerk.
// - The reference orientation is advanced by the rotation vector Ruckig integrated over the tick
//   (angular velocity in the base frame, so it is applied from the left).
// The reference runs open loop (never reset to the measured pose), so in contact the impedance
// force still is Kp * (reference - measured) and the reference keeps converging to the target.
class CartesianReferenceGenerator
{
  public:
    static constexpr double kControlPeriod = 0.001;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    // Start the reference at rest at the current end-effector pose
    void reset(const Eigen::Vector3d& current_pos, const Eigen::Quaterniond& current_quat,
               const CartesianFeedforwardConfig& config)
    {
        position_gain_ = config.reference_position_gain;
        braking_ = config.reference_braking;
        velocity_filter_time_ = config.reference_velocity_filter_time;
        filtered_twist_.setZero();
        ref_quat_ = current_quat.normalized();
        input_ = ruckig::InputParameter<6>();
        input_.control_interface = ruckig::ControlInterface::Velocity;
        // each axis reaches its target velocity as fast as its own limits allow (no coupling)
        input_.synchronization = ruckig::Synchronization::None;
        for (size_t i = 0; i < 6; i++)
        {
            input_.current_position[i] = i < 3 ? current_pos[i] : 0.0;
            input_.max_velocity[i] = config.max_velocity[i];
            input_.max_acceleration[i] = config.max_acceleration[i];
            input_.max_jerk[i] = config.max_jerk[i];
        }
        // braking deceleration along any direction: the smallest axis limit
        for (size_t k = 0; k < 2; k++)
        {
            brake_acc_[k] = std::min({config.max_acceleration[3 * k], config.max_acceleration[3 * k + 1],
                                      config.max_acceleration[3 * k + 2]});
        }
        otg_.reset();
    }

    // Advance the reference by `steps` control periods (libfranka reports >1 after lost packets)
    void update(const Eigen::Vector3d& target_pos, const Eigen::Quaterniond& target_quat,
                const Vector6d& target_twist, uint64_t steps, Eigen::Vector3d& ref_pos,
                Eigen::Quaterniond& ref_quat, Vector6d& ref_twist, Vector6d& ref_acc)
    {
        const double alpha = kControlPeriod / (velocity_filter_time_ + kControlPeriod);
        for (uint64_t k = 0; k < steps; k++)
        {
            filtered_twist_ += alpha * (target_twist - filtered_twist_);
            const Eigen::Vector3d pos_err =
                target_pos - Eigen::Vector3d(input_.current_position[0], input_.current_position[1],
                                             input_.current_position[2]);
            const Eigen::Vector3d rot_err = rotationVector(target_quat * ref_quat_.inverse());
            Vector6d desired;
            desired << filtered_twist_.head<3>() + correction(pos_err, brake_acc_[0]),
                filtered_twist_.tail<3>() + correction(rot_err, brake_acc_[1]);
            for (size_t part = 0; part < 2; part++)
            {
                // Ruckig rejects a target velocity outside the limits; scale the whole linear /
                // angular part so its direction is kept
                double scale = 1.0;
                for (size_t i = 3 * part; i < 3 * part + 3; i++)
                {
                    const double v_max = 0.99 * input_.max_velocity[i];
                    if (std::abs(desired[i]) > v_max)
                    {
                        scale = std::min(scale, v_max / std::abs(desired[i]));
                    }
                }
                desired.segment<3>(3 * part) *= scale;
            }
            for (size_t i = 0; i < 6; i++)
            {
                input_.target_velocity[i] = desired[i];
            }
            if (otg_.update(input_, output_) < 0)
            {
                // should not happen with valid limits; brake the reference instead
                input_.target_velocity.fill(0.0);
                if (otg_.update(input_, output_) < 0)
                {
                    break;  // keep the last reference
                }
            }
            const Eigen::Vector3d rot_step(output_.new_position[3] - input_.current_position[3],
                                           output_.new_position[4] - input_.current_position[4],
                                           output_.new_position[5] - input_.current_position[5]);
            ref_quat_ = (rotationFromVector(rot_step) * ref_quat_).normalized();
            output_.pass_to_input(input_);
            // the angular "position" is only used for the per-tick increment; keep it small
            for (size_t i = 3; i < 6; i++)
            {
                input_.current_position[i] = 0.0;
            }
        }
        ref_pos = Eigen::Vector3d(input_.current_position[0], input_.current_position[1],
                                  input_.current_position[2]);
        ref_quat = ref_quat_;
        for (size_t i = 0; i < 6; i++)
        {
            ref_twist[i] = input_.current_velocity[i];
            ref_acc[i] = input_.current_acceleration[i];
        }
    }

  private:
    // Velocity that pulls the reference onto the target along the error direction
    Eigen::Vector3d correction(const Eigen::Vector3d& error, double brake_acc) const
    {
        const double e = error.norm();
        if (e < 1e-12)
        {
            return Eigen::Vector3d::Zero();
        }
        const double speed = std::min(position_gain_ * e, std::sqrt(2.0 * braking_ * brake_acc * e));
        return (speed / e) * error;
    }

    double position_gain_{30.0};
    double braking_{0.5};
    double velocity_filter_time_{0.02};
    std::array<double, 2> brake_acc_{};
    Vector6d filtered_twist_{Vector6d::Zero()};
    Eigen::Quaterniond ref_quat_{Eigen::Quaterniond::Identity()};
    ruckig::Ruckig<6> otg_{kControlPeriod};
    ruckig::InputParameter<6> input_;
    ruckig::OutputParameter<6> output_;
};
