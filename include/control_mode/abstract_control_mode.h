#ifndef ABSTRACT_CONTROL_MODE_H
#define ABSTRACT_CONTROL_MODE_H

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <memory>//for std::shared_ptr
#include <mutex>
//todo:reform and check the leadter state get and the is_running
class AbstractControlMode {
public:
    // Virtual destructor for proper cleanup in derived classes
    virtual ~AbstractControlMode() = default;
    // Pure virtual public functions
    //virtual void initialize(const RobotState& initial_state);
    virtual void start() = 0;
    virtual void stop() = 0;
    // Get the mode ID for this control mode
    virtual int getModeID() const = 0; // Return the mode ID as an integer
    void setRobot(std::shared_ptr<franka::Robot> robot) {
        robot_ = std::move(robot);
    }
    void setModel(std::shared_ptr<franka::Model> model) {
        model_ = std::move(model);
    }
    // Set the current state of the robot into variable current_state_
    void setCurrentState(const franka::RobotState& state) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = state;
    }
    
    // get current state of robot
    virtual franka::RobotState getRobotState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }

    void setLeaderState(const franka::RobotState& leader_state) {
        std::lock_guard<std::mutex> lock(leader_mutex_);
        leader_state_ = std::make_shared<franka::RobotState>(leader_state);
    }

    std::shared_ptr<const franka::RobotState> getLeaderState() const {
        std::lock_guard<std::mutex> lock(leader_mutex_);
        return leader_state_;
    }
    //for mode use:
    // auto leader_ptr = getLeaderState();
    //     

    // const franka::RobotState& leader = *leader_state_ptr;
    //franka::Torques tau = computeTorqueFromLeader(leader);



protected:
    // Protected constructor to prevent direct instantiation
    AbstractControlMode() = default;
    // Protected setup function for derived classes
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    franka::RobotState current_state_;
    mutable std::mutex state_mutex_;
    bool is_running_ = false;
    std::shared_ptr<franka::RobotState> leader_state_;
    mutable std::mutex leader_mutex_; // Protects access to leader_state_
};

#endif // ABSTRACT_CONTROL_MODE_H