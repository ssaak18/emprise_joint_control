#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from sensor_msgs.msg import JointState

class KinovaController:
    def __init__(self):
        rospy.init_node("kinova_joint_state_controller", anonymous=True)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.check_inital_pos)
        self.kinova_joint_state_sub = rospy.Subscriber("/joint_states", JointState, queue_size=10)

        # torque controller?

        # get measured state and then have controller move joint to new posistion from input 
        
    def check_inital_pos(self, joint_state_msg):
        robot_init_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        pos = np.array(joint_state_msg.posistions)
        error = 0.025 #radians
        return np.all(np.abs((robot_init_pos - pos)) <= error)
        


class SimController:
    def __init__(self):
        rospy.init_node("sim_joint_state_controller", anonymous=True)
        self.kinova_joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)

        # need robot inputs/outputs

    def joint_input_callback(self, joint_state_msg):
        self.kinova_joint_state_pub.publish(joint_state_msg)

    def start(self):
        rospy.loginfo("Starting Sim Controller")
        rospy.spin()

def start(safe_to_start):
    if safe_to_start:  
        if len(sys.argv) < 1:
            rospy.loginfo("Usage: rosrun joint_controller_pkg controller.py <mode>")
            return

        mode = sys.argv[1]

        if mode == "sim":
            controller = SimController()
            controller.start()
        elif mode == "kinova":
            controller = KinovaController()
            controller.start()

safe_to_start = True
if __name__ == "__main__": 
    start(safe_to_start)