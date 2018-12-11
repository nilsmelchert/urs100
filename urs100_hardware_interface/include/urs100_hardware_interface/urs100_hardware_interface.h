#ifndef ROS_CONTROL__URS100_HARDWARE_INTERFACE_H
#define ROS_CONTROL__URS100_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#include "urs100_driver/Urs100.h"
#include "urs100_hardware_interface/urs100_hardware.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace urs100_hardware_interface
{
    // Documentation says 0.5 mdeg encoder resolution which is 0.00174533 rad
    static const double POSITION_STEP_FACTOR = 0.002;
    static const double VELOCITY_STEP_FACTOR = 0.002; //TODO: Figure out what this is

    class Urs100HardwareInterface : public urs100_hardware_interface::urs100Hardware
    {
    public:
        Urs100HardwareInterface(ros::NodeHandle& nh);
        ~Urs100HardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);

    protected:
        urs100_driver::Urs100 Urs100Stage; //"/dev/ttyUSB0" ,57600, false
        ros::NodeHandle nh_;
        JointLimits limits_;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;
        PositionJointInterface positionJointInterface;
        PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;

        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    };
}

#endif