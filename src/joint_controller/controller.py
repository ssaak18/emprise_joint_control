#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

class SafetySwitchControlled:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
        self.kinova_joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def joint_input_callback(self, joint_state_msg):
        in_bounds_pos = self.check_joint_limit(joint_state_msg.position)
        new_joint_state_msg = JointState()
        new_joint_state_msg.header.stamp = rospy.Time.now()
        new_joint_state_msg.name = [f"joint_{i+1}" for i in range(len(in_bounds_pos))]
        new_joint_state_msg.position = in_bounds_pos
        self.kinova_joint_state_pub.publish(new_joint_state_msg)

    def check_joint_limit(self, joint_pos):
        """ Ensures joint values remain in Kinova joint ranges """
        check_pos = np.array(joint_pos)
        joint_min = np.array([-np.inf, -2.25, -np.inf, -2.58, -np.inf, -2.1, -np.inf])
        joint_max = np.array([np.inf, 2.25, np.inf, 2.58, np.inf, 2.1, np.inf])
        return np.clip(check_pos, joint_min, joint_max)

    def start(self):
        rospy.loginfo(f"Starting {self.__class__.__name__}")
        rospy.spin()

class KinovaController(SafetySwitchControlled):
    def __init__(self):
        super().__init__("kinova_controller")
        self.kinova_joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.check_initial_pos)
        self.good_to_start = False

        rospy.loginfo("Move exoskeleton to initial position")
        while not rospy.is_shutdown() and not self.good_to_start:
            rospy.sleep(0.2)

        rospy.loginfo("Initial position reached!")

    def check_initial_pos(self, joint_state_msg):
        """Checks if the robot is in the initial position before starting."""
        robot_init_pos = np.array([0.0] * 7)
        error = 0.025  # radians
        self.good_to_start = np.all(np.abs(robot_init_pos - joint_state_msg.position) <= error)

    def joint_input_callback(self, input_state_msg):
        """Publishes joint states only if the robot is ready to start."""
        if self.good_to_start:
            super().joint_input_callback(input_state_msg)

class SimController(SafetySwitchControlled):
    def __init__(self):
        super().__init__("sim_controller")

if __name__ == "__main__": 
    mode = rospy.get_param("mode", "sim")

    if mode == "sim":
        controller = SimController()
    elif mode == "kinova":
        controller = KinovaController()

    controller.start()
