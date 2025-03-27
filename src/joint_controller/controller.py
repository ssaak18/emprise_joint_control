#!/usr/bin/env python3
import rospy
import sys
from pynput import keyboard
import numpy as np
from sensor_msgs.msg import JointState

# class SafteySwitchControlled:
    
class KinovaController:
    def __init__(self):
        rospy.init_node("kinova_joint_state_controller", anonymous=True)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
        self.kinova_joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.check_inital_pos)
        self.kinova_pub = rospy.Publisher("/joint_states", JointState, queue_size=10) #replace with diff controller
        
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.is_publishing = False
        self.good_to_start = False
        
        # move the robot to the straight pos (all joints at 0.0) before starting!
        
        rospy.loginfo("Move exoskeleton to initial position")
        while not rospy.is_shutdown() and not self.good_to_start:
            rospy.sleep(0.2) 
            
        rospy.loginfo("Initial position reached!")
            
    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"joint_state publishing {'started' if self.is_publishing else 'stopped'}!")
        except AttributeError:
            pass
            
    def joint_input_callback(self, input_state_msg):
        if self.is_publishing and self.good_to_start:
            self.kinova_joint_state_pub.publish(input_state_msg)
        
    def check_inital_pos(self, joint_state_msg):
        robot_init_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        error = 0.025 #radians
        self.good_to_start = np.all(np.abs((robot_init_pos - joint_state_msg.position)) <= error)
    
    def start(self):
        rospy.loginfo("Starting Sim Controller")
        rospy.spin()
        


class SimController:
    def __init__(self):
        rospy.init_node("sim_joint_state_controller", anonymous=True)
        self.kinova_joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.joint_input_sub = rospy.Subscriber("/input_states", JointState, self.joint_input_callback)
        self.is_publishing = False
        
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

        # need robot inputs/outputs

    def joint_input_callback(self, joint_state_msg):
        if self.is_publishing:
            self.kinova_joint_state_pub.publish(joint_state_msg)
            
    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"joint_state publishing {'started' if self.is_publishing else 'stopped'}!")
        except AttributeError:
            pass

    def start(self):
        rospy.loginfo("Starting Sim Controller")
        rospy.spin()
        
if __name__ == "__main__": 
    
    mode = rospy.get_param("mode", "test")
    robot_controller = rospy.get_param("use_robot_controllersensor", "pos")
    
    if mode == "sim":
        controller = SimController()
        controller.start()
    elif mode == "kinova":
        controller = KinovaController()
        controller.start()