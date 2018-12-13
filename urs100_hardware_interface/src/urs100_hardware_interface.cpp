#include <sstream>
#include "urs100_hardware_interface/urs100_hardware_interface.h"
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "urs100_driver/Urs100.h"

using namespace hardware_interface;

using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace urs100_hardware_interface {

//    urs100_hardware_interface::urs100HardwareInterface()
    Urs100HardwareInterface::Urs100HardwareInterface(ros::NodeHandle &nh) : nh_(nh),
                                                                            Urs100Stage("/dev/ttyUSB0", 57600, false) {
        // TODO: get device name and baud rate from parameter server

        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this,
                                                                            nh_)); // Note: IDE might not able to resolve double inheritation for controller manager

        nh_.param("/urs100/hardware_interface/loop_hz", loop_hz_, 0.1);
        ROS_DEBUG_STREAM_NAMED("constructor", "Using loop freqency of " << loop_hz_ << " hz");
        ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &Urs100HardwareInterface::update, this);

        ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
    }

    Urs100HardwareInterface::~Urs100HardwareInterface() {
        // Empty destructor
    }

    void Urs100HardwareInterface::init() {
        // Set velocity and effort to 0 since they are nto used.
        joint_velocity_ = 0.0;
        joint_effort_ = 0.0;

        // Get joint names
        nh_.getParam("/urs100/hardware_interface/joints", joint_names_);

        // Initialize Controller
        // Create joint state interface
        JointStateHandle jointStateHandle(joint_names_[0], &joint_position_, &joint_velocity_, &joint_effort_);
        joint_state_interface_.registerHandle(jointStateHandle);


        // Create position joint interface
        JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
        JointLimits limits;
        SoftJointLimits softLimits;

        getJointLimits(joint_names_[0], nh_, limits);
        limits_ = limits;
        PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
        positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        position_joint_interface_.registerHandle(jointPositionHandle);

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
    }

    void Urs100HardwareInterface::read() {
        joint_position_ = this->Urs100Stage.getPosition();
    }

    void Urs100HardwareInterface::write(ros::Duration elapsed_time) {
//        TODO: Implement a proper saturation with enforce limits:
//        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
// This is just a temporary solution
        double position;
        if (joint_position_command_ > limits_.max_position) {
            position = limits_.max_position;
        } else if (joint_position_command_ < limits_.min_position) {
            position = limits_.min_position;
        } else {
            position = joint_position_command_;
        }

        Urs100Stage.setPosition(position);
    }

    void Urs100HardwareInterface::update(const ros::TimerEvent &e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        this->read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        this->write(elapsed_time_);
    }

}
