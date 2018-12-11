#ifndef ROS_CONTROL__URS100_HARDWARE_H
#define ROS_CONTROL__URS100_HARDWARE_H


#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace urs100_hardware_interface
{
    /// \brief Hardware interface for a robot
    class urs100Hardware : public hardware_interface::RobotHW
    {
    protected:
        // Interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
//        hardware_interface::VelocityJointInterface velocity_joint_interface_;
//        hardware_interface::EffortJointInterface effort_joint_interface_;
//
//        joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
//        joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
//        joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
//        joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
//        joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

        // Shared memory
        std::vector<std::string> joint_names_;

        double joint_position_;
        double joint_velocity_;
        double joint_effort_;
        double joint_position_command_;

        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;

    }; // class
}

#endif