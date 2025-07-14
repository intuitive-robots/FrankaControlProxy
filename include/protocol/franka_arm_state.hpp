
#ifndef FRANKA_ARM_STATE_HPP
#define FRANKA_ARM_STATE_HPP
#include <array>
#include <cstdint>
#include <vector>
#include <cstddef>
#include <string>
#include <chrono>
#include <mutex>
#include <franka/robot_state.h>

namespace protocol {

class FrankaArmState {
	public:
	// RobotState Payload 636 byte
	static constexpr size_t kSize = 636;
	uint32_t timestamp_ms;                      //timestamp [ms]
	std::array<double, 16> O_T_EE;              // 4×4 homogeneous EE pose
	std::array<double, 16> O_T_EE_d;            // 4×4 homogeneous EE target pose
	// Joint space data (7 DOF)
	std::array<double, 7> q;                    // joint angles actual [rad]
	std::array<double, 7> q_d;                  // joint angles target [rad]
	std::array<double, 7> dq;                   // joint velocities actual [rad s⁻¹]
	std::array<double, 7> dq_d;                 // joint velocities target [rad s⁻¹]
	std::array<double, 7> tau_ext_hat_filtered; // filtered external torque estimate [N m]
	std::array<double, 6> O_F_ext_hat_K;        // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame.[N N N Nm Nm Nm]
	std::array<double, 6> K_F_ext_hat_K;        // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame.[N N N Nm Nm Nm]
	// Thread safety
    //mutable std::mutex mutex; //will be implicitly deleted
	// Encode 
	std::vector<uint8_t> encode() const;
	
	// Decode
	static FrankaArmState decode(const uint8_t* buffer, size_t size);

	//Upstate from SDK
	static FrankaArmState fromRobotState(const franka::RobotState& robot_state);

	//Transfer to SDK(for follower)
	franka::RobotState toRobotState() const;

};



}  // namespace protocol

#endif // FRANKA_ARM_STATE_HPP