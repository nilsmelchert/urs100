import rospy
from urs100_hardware_interface.srv import SetJointPos, SetJointPosRequest, SetJointPosResponse
from std_msgs.msg import Float64


class Urs100JointPublisher:

    def __init__(self):
        rospy.init_node('urs_100_set_point_ss', anonymous=True)
        self.pub = rospy.Publisher('joint_urs100_set_point_pub', Float64, queue_size=10)
        self.joint_pos_server()

    def send_command(self, req):
        try:
            self.pub.publish(req.joint_pos)
            return True
        except:
            return False

    def joint_pos_server(self):
        s = rospy.Service('urs_100_set_point', SetJointPos, self.send_command)
        rospy.loginfo("Ready to send joint positions to rotary stage urs100.")
        rospy.spin()


if __name__ == '__main__':
    JointPublisher = Urs100JointPublisher()
