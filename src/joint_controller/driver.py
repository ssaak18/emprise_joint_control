#!/usr/bin/env python3
import rospy
import math
from dynamixel_sdk import * 
from pynput import keyboard
import sys
import numpy as np
from sensor_msgs.msg import JointState

class InputDriver:
    
    def __init__(self, node_name):
        self.joint_input_pub = rospy.Publisher("/input_states", JointState, queue_size=10)
        self.is_publishing = False

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        """Toggle joint state publishing when 'z' is pressed."""
        try:
            if key.char == 'z':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"Joint state publishing {'started' if self.is_publishing else 'stopped'}!")
        except AttributeError:
            pass
        
    def make_joint_msg(self, pos, vel):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(pos))]
        joint_state_msg.position = pos
        joint_state_msg.velocity = vel
        return joint_state_msg

    def start(self):
        """Start the driver and continuously publish joint positions."""
        rospy.loginfo(f"Starting {self.__class__.__name__}")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.get_joint_pos()
            rate.sleep()

class TestDriver(InputDriver):
    """Simulation driver"""

    def __init__(self):
        super().__init__("test_driver")

    def get_joint_pos(self):
        joint_pos = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos, joint_vel))

class KeyDriver(InputDriver):
    """Keyboard driver"""
    
    global joint_pos 
    joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("key_driver")
        
    def on_key_press(self, key):
        """Basic keyboard controls; keys above/below each other (ex. q/a) control +/-.1 for the 7 joints """
        try:
            if key.char == 'z':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"Joint state publishing {'started' if self.is_publishing else 'stopped'}!")
            elif key.char == 'q':
                joint_pos[0] += .1
            elif key.char == 'a':
                joint_pos[0] -= .1
            elif key.char == 'w':
                joint_pos[1] += .1
            elif key.char == 's':
                joint_pos[1] -= .1
            elif key.char == 'e':
                joint_pos[2] += .1
            elif key.char == 'd':
                joint_pos[2] -= .1
            elif key.char == 'r':
                joint_pos[3] += .1
            elif key.char == 'f':
                joint_pos[3] -= .1
            elif key.char == 't':
                joint_pos[4] += .1
            elif key.char == 'g':
                joint_pos[4] -= .1
            elif key.char == 'y':
                joint_pos[5] += .1
            elif key.char == 'h':
                joint_pos[5] -= .1
            elif key.char == 'u':
                joint_pos[6] += .1
            elif key.char == 'j':
                joint_pos[6] -= .1
        except AttributeError:
            pass

    def get_joint_pos(self):
        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos, self.joint_vel))

class ExoDriver(InputDriver):
    """Driver for controlling real exoskeleton hardware via Dynamixel motors"""
    zero_offsets = None

    PORT = "/dev/ttyUSB0"
    BAUDRATE = 1000000  # set for all servos
    PROTOCOL_VERSION = 2.0
    PRESENT_VEL_ADDR = 128
    PRESENT_POS_ADDR = 132  # Position address in control table (0-4095)
    MAX_POS = 4095.0

    def __init__(self, motor_ids):
        super().__init__("exo_driver")
        self.motor_ids = motor_ids
        self.portHandler = PortHandler(self.PORT)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort() and self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.loginfo("Connected to Dynamixel")
        else:
            rospy.logerr("Failed to connect to Dynamixel")
            exit()

    def get_joint_pos(self):
        joint_pos = []
        joint_vel = []

        for motor_id in self.motor_ids:
            dxl_pos, dxl_comm_result1, _ = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_POS_ADDR)
            dxl_vel, dxl_comm_result2, _ = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_POS_ADDR)

            if dxl_comm_result1 != COMM_SUCCESS or dxl_comm_result2 != COMM_SUCCESS:
                rospy.logerr(f"Motor {motor_id}: Read error {dxl_comm_result1, dxl_comm_result2}")
                joint_pos.append(0.0)
                continue

            dxl_pos = self.unsigned_to_signed(dxl_pos)
            pos_rad = dxl_pos / self.MAX_POS * 2 * math.pi
            if motor_id % 2 == 1:
                pos_rad = -1 * pos_rad
            joint_pos.append(pos_rad)
            joint_vel.append(dxl_vel)

        # Initialize zero offsets once
        if self.zero_offsets is None:
            self.zero_offsets = joint_pos
            rospy.loginfo("Initialized zero offsets.")

        relative_joint_pos = [
            curr - zero for curr, zero in zip(joint_pos, self.zero_offsets)
        ]

        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(relative_joint_pos, joint_vel))

    def unsigned_to_signed(self, val, bits=32):
        if val >= 2**(bits - 1):
            val -= 2**bits
        return val

def start():
    rospy.init_node("driver_node", anonymous=True)

    mode = rospy.get_param("~driver_mode", "exo")
    rospy.loginfo(f"[DRIVER] mode param: {mode}")

    if mode == "exo":
        motor_ids = [1,2,3,4,5,6,7] # set for servos in wizard
        exo_driver = ExoDriver(motor_ids)
        exo_driver.start()
    elif mode == "test":
        test_driver = TestDriver()
        test_driver.start()
    elif mode == "key":
        key_driver = KeyDriver()
        key_driver.start()

if __name__ == "__main__": 
    start()
