#!/usr/bin/env python3
import rospy
import math
from dynamixel_sdk import * 
from pynput import keyboard
import sys
from sensor_msgs.msg import JointState

class InputDriver:
    
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)
        self.joint_input_pub = rospy.Publisher("/input_states", JointState, queue_size=10)
        self.is_publishing = False

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        """Toggle joint state publishing when 's' is pressed."""
        try:
            if key.char == 's':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"Joint state publishing {'started' if self.is_publishing else 'stopped'}!")
        except AttributeError:
            pass
        
    def make_joint_msg(self, pos):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(pos))]
        joint_state_msg.position = pos
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
        
        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos))

class ExoDriver(InputDriver):
    """Driver for controlling real exoskeleton hardware via Dynamixel motors"""

    PORT = "/dev/ttyUSB0"
    BAUDRATE = 57600  # Factory default setting
    PROTOCOL_VERSION = 2.0
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
        """Reads and publishes joint positions from Dynamixel motors"""
        joint_pos = []
        
        for motor_id in self.motor_ids:
            dxl_pos, dxl_comm_result = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_POS_ADDR)

            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr(f"Motor {motor_id}: Read error {dxl_comm_result}")
                continue
            
            joint_pos.append(dxl_pos / self.MAX_POS * 2 * math.pi)

        if self.is_publishing:
            self.joint_input_pub.publish(super().make_joint_msg(joint_pos))

def start():
    mode = rospy.get_param("mode", "test")

    if mode == "exo":
        exo_driver = ExoDriver(list(map(int, sys.argv[1].split(','))))
        exo_driver.start()
    elif mode == "test":
        test_driver = TestDriver()
        test_driver.start()

if __name__ == "__main__": 
    start()
