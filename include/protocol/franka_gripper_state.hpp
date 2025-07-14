#ifndef FRANKA_GRIPPER_STATE_HPP
#define FRANKA_GRIPPER_STATE_HPP
#include <array>
#include <cstdint>
#include <vector>
#include <cstddef>
#include <string>
#include <chrono>
#include <mutex>
#include <franka/gripper_state.h>

namespace protocol {

class FrankaGripperState {
	public:
	// GripperState Payload 23byte
	static constexpr size_t kSize = 23;
	uint32_t timestamp_ms; //4      
	double width;//8
	double max_width;//8
	bool grasped;//1
	uint16_t temperature;//2

	// Thread safety
    //mutable std::mutex mutex; //will be implicitly deleted
	// Encode 
	std::vector<uint8_t> gripper_encode() const;
	
	// Decode
	static FrankaGripperState gripper_decode(const uint8_t* buffer, size_t size);

	//Upstate from SDK
	static FrankaGripperState fromGripperState(const franka::GripperState& gripper_state);
	 
	//Transfer to SDK(for follower)
	franka::GripperState toGripperState() const;
};



}  // namespace protocol

#endif // FRANKA_GRIPPER_STATE_HPP