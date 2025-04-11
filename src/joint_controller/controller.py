#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from pynput import keyboard

class SafetySwitchControlled:
    
    old_pos = []
    safe_speed = True
    
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
        self.kinova_joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        """Toggle safe_speed when 'c' is pressed."""
        try:
            if key.char == 'c':
                self.safe_speed = True
        except AttributeError:
            pass

    def joint_input_callback(self, joint_state_msg):
        if self.old_pos.__len__ == 0:
            self.old_pos = joint_state_msg.position
            
        in_bounds_pos = self.safe_move(joint_state_msg.velocity, joint_state_msg.position)
            
        # in_bounds_pos = self.check_joint_limit(joint_state_msg.position)
        new_joint_state_msg = JointState()
        new_joint_state_msg.header.stamp = rospy.Time.now()
        new_joint_state_msg.name = [f"joint_{i+1}" for i in range(len(in_bounds_pos))]
        new_joint_state_msg.position = in_bounds_pos
        new_joint_state_msg.velocity = joint_state_msg.velocity
        self.kinova_joint_state_pub.publish(new_joint_state_msg)
        
    def safe_move(self, joint_vel, new_pos):
        
        vel = np.array(joint_vel)
        self.safe_speed = self.safe_speed and np.all((vel >= -100) & (vel <= 100))
        
        if self.safe_speed:
            self.old_pos = self.check_joint_limit(new_pos)
        else:
            rospy.loginfo("Sudden movement detected! Press 'c' if this was expected.")
            
        return self.old_pos
        

    def check_joint_limit(self, joint_pos):
        """ Ensures joint values remain in Kinova joint ranges """
        max = float('inf') 
        min = -float('inf') 
        check_pos = np.array(joint_pos)
        joint_min = np.array([min, -2.25, min, -2.58, min, -2.1, min])
        joint_max = np.array([max, 2.25, max, 2.58, max, 2.1, max])
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
