#!/usr/bin/env python

import rospy
from urs100_hardware_interface.srv import SetJointPos, SetJointPosRequest
from std_msgs.msg import Float64


class Urs100JointPublisher:

    def __init__(self):
        rospy.init_node('urs_100_set_point_ss', anonymous=True)
        self.pub = rospy.Publisher('/urs100/controller/position/joint_urs100/command', Float64, queue_size=10)
        self.joint_pos_server()

    def send_command(self, req):
            self.pub.publish(req.joint_pos)
            return []

    def joint_pos_server(self):
        s = rospy.Service('urs_100_set_point', SetJointPos, self.send_command)
        rospy.loginfo("Ready to send joint positions to rotary stage urs100.")
        rospy.spin()


if __name__ == '__main__':
    JointPublisher = Urs100JointPublisher()
