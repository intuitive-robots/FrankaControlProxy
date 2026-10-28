#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ruckig/ruckig.hpp>

#include "utils/config_file_reader.hpp"
#include "utils/robot_model.hpp"

// Desired joint position and feed-forward velocity of one command, stored together so the
// control loop never mixes position and velocity of different commands.
struct JointTarget
{
    JointPosition pos{JointPosition::Zero()};
    JointVelocity vel{JointVelocity::Zero()};
    std::chrono::steady_clock::time_point stamp{};
};

struct JointFeedforwardConfig
{
    // Scale of the commanded velocity used as velocity feed-forward and for extrapolation (0 = off)
    double velocity_feedforward{1.0};
    // Scale of the reference acceleration fed through the inertia M(q) (0 = off)
    double acceleration_feedforward{0.0};
    // Shift the target position along the commanded velocity between commands
    bool extrapolate_position{true};
    // Velocity feed-forward and extrapolation stop when the last command is older than this [s]
    double feedforward_timeout{0.05};
    // Reference trajectory (see JointReferenceGenerator):
    // gain [1/s] of the position loop that pulls the reference onto the target
    double reference_position_gain{30.0};
    // fraction of max_acceleration the position loop plans to brake with (rest is jerk margin)
    double reference_braking{0.5};
    // low-pass time constant [s] of the commanded velocity (GELLO velocity is a noisy difference)
    double reference_velocity_filter_time{0.02};
    // Limits of the reference trajectory. Leader motions beyond them lag instead of reaching the robot.
    std::array<double, 7> max_velocity{};
    std::array<double, 7> max_acceleration{};
    std::array<double, 7> max_jerk{};

    void read(const ConfigFileReader& reader)
    {
        velocity_feedforward = reader.getValue<double>("velocity_feedforward", 1.0);
        acceleration_feedforward = reader.getValue<double>("acceleration_feedforward", 0.0);
        extrapolate_position = reader.getValue<bool>("extrapolate_position", true);
        feedforward_timeout = reader.getValue<double>("feedforward_timeout", 0.05);
        reference_position_gain = reader.getValue<double>("reference_position_gain", 30.0);
        reference_braking = reader.getValue<double>("reference_braking", 0.5);
        reference_velocity_filter_time = reader.getValue<double>("reference_velocity_filter_time", 0.02);
        // required: every preset has to state its limits
        max_velocity = reader.getArray<double, 7>("max_velocity");
        max_acceleration = reader.getArray<double, 7>("max_acceleration");
        max_jerk = reader.getArray<double, 7>("max_jerk");
    }
};

// Target position and velocity of the latest command for the current control tick. Without a
// commanded velocity this reduces to the plain position target with zero target velocity.
inline void jointTarget(const JointTarget& target, const JointFeedforwardConfig& config,
                        JointPosition& target_pos, JointVelocity& target_vel)
{
    const double cmd_age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - target.stamp).count();
    const JointVelocity vel = config.velocity_feedforward * target.vel;
    target_vel = cmd_age < config.feedforward_timeout ? vel : JointVelocity::Zero();
    target_pos = target.pos;
    if (config.extrapolate_position)
    {
        // Capped at the timeout so the target stops (and does not jump back) when commands stop
        target_pos += vel * std::min(cmd_age, config.feedforward_timeout);
    }
}

// Reference trajectory for the controller, so it never sees the raw (stepping, noisy, possibly
// jumping) leader commands. Every control tick:
// - desired velocity = filtered commanded velocity + position loop on the target error. The loop
//   is linear for small errors (precise motions pass with little lag) and limited to
//   sqrt(2 * braking * a_max * |error|) for large ones, so the reference can always stop at the
//   target without overshoot.
// - Ruckig (velocity interface) reaches that velocity within max_velocity/acceleration/jerk.
// Ruckig's position interface is not used on purpose: replanning every tick towards a noisy
// moving target makes the time-optimal profile chatter at full jerk (0.5-20 deg reference error
// in tests). This structure gave <0.01 deg on clean and ~0.1-0.25 deg on noisy slow motions,
// no step overshoot, and bounded acceleration/jerk for any command.
class JointReferenceGenerator
{
  public:
    static constexpr double kControlPeriod = 0.001;

    // Start the reference at rest at the current robot position
    void reset(const JointPosition& current_pos, const JointFeedforwardConfig& config)
    {
        position_gain_ = config.reference_position_gain;
        braking_ = config.reference_braking;
        velocity_filter_time_ = config.reference_velocity_filter_time;
        filtered_vel_.setZero();
        input_ = ruckig::InputParameter<7>();
        input_.control_interface = ruckig::ControlInterface::Velocity;
        // each joint reaches its target velocity as fast as its own limits allow (no coupling)
        input_.synchronization = ruckig::Synchronization::None;
        for (size_t i = 0; i < 7; i++)
        {
            input_.current_position[i] = current_pos[i];
            input_.max_velocity[i] = config.max_velocity[i];
            input_.max_acceleration[i] = config.max_acceleration[i];
            input_.max_jerk[i] = config.max_jerk[i];
        }
        otg_.reset();
    }

    // Advance the reference by `steps` control periods (libfranka reports >1 after lost packets)
    void update(const JointPosition& target_pos, const JointVelocity& target_vel, uint64_t steps,
                JointPosition& ref_pos, JointVelocity& ref_vel, JointAcceleration& ref_acc)
    {
        const double alpha = kControlPeriod / (velocity_filter_time_ + kControlPeriod);
        for (uint64_t k = 0; k < steps; k++)
        {
            filtered_vel_ += alpha * (target_vel - filtered_vel_);
            for (size_t i = 0; i < 7; i++)
            {
                const double error = target_pos[i] - input_.current_position[i];
                const double braking_vel =
                    std::sqrt(2.0 * braking_ * input_.max_acceleration[i] * std::abs(error));
                const double correction =
                    std::copysign(std::min(position_gain_ * std::abs(error), braking_vel), error);
                // Ruckig rejects a target velocity outside the limits
                const double v_max = 0.99 * input_.max_velocity[i];
                input_.target_velocity[i] = std::clamp(filtered_vel_[i] + correction, -v_max, v_max);
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
            output_.pass_to_input(input_);
        }
        ref_pos = JointPosition::Map(input_.current_position.data());
        ref_vel = JointVelocity::Map(input_.current_velocity.data());
        ref_acc = JointAcceleration::Map(input_.current_acceleration.data());
    }

  private:
    double position_gain_{30.0};
    double braking_{0.5};
    double velocity_filter_time_{0.02};
    JointVelocity filtered_vel_{JointVelocity::Zero()};
    ruckig::Ruckig<7> otg_{kControlPeriod};
    ruckig::InputParameter<7> input_;
    ruckig::OutputParameter<7> output_;
};
