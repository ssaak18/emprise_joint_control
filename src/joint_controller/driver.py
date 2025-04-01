#!/usr/bin/env python3
import rospy
import math
from dynamixel_sdk import * 
from pynput import keyboard
import sys
from sensor_msgs.msg import JointState

class TestDriver:
    def __init__(self):

        rospy.init_node("driver", anonymous=True)
        self.joint_input_pub = rospy.Publisher("/input_states", JointState, queue_size=10)
        
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.is_publishing = False
        
    def on_key_press(self, key):
        try:
            if key.char == 's':
                self.is_publishing = not self.is_publishing
                rospy.loginfo(f"joint_state publishing {'started' if self.is_publishing else 'stopped'}!")
        except AttributeError:
            pass

    def get_joint_pos(self):
        joint_pos = [0.0,1.0,2.0,3.0,4.0,5.0,6.0]

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(joint_pos))]
        joint_state_msg.position = joint_pos
        
        if self.is_publishing:
            self.joint_input_pub.publish(joint_state_msg)

    def start(self):
        rospy.loginfo("Starting Test Driver")
        # rospy.spin()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.get_joint_pos()
            # print("in")
            rate.sleep()

class ExoDriver:

    port="/dev/ttyUSB0"
    baudrate= 57600 # factory default setting

    PROTOCOL_VERSION = 2.0 # for DYNAMIXEL XC330-M288-T (and other newer models)
    PRESENT_POS_ADDR = 132 # in control table (0-4095)
    MAX_POS = 4095.0

    init_joint_states = [0,0,0,0,0,0,0]

    def __init__(self, motor_ids):
        self.motor_ids = motor_ids
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.motor_ids = motor_ids

        rospy.init_node("driver", anonymous=True)
        self.joint_input_pub = rospy.Publisher("/input_states", JointState, queue_size=10)

        # open port
        if self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate):
            rospy.loginfo("Connected to Dynamixel")
        else:
            rospy.logger("Failed to connect")
            exit()

        # TODO WRITE INITIAL JOINT STATE TRANSLATION

    def get_joint_pos(self):
        joint_pos = []

        for motor_id in self.motor_ids:

            dxl_pos, dxl_comm_result = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, self.PRESENT_POS_ADDR)

            if dxl_comm_result != COMM_SUCCESS:
                rospy.logger(f"Motor {motor_id}: Read error {dxl_comm_result}")
                continue
            
            joint_pos.append(dxl_pos / self.MAX_POS * 2 * math.pi)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f"joint_{i+1}" for i in range(len(self.motor_ids))]
        joint_state_msg.position = joint_pos #array of floats
        self.kinova_joint_state_pub.publish(joint_state_msg)
        
        self.joint_input_pub.publish(joint_state_msg)
    
    def start(self):
        rospy.loginfo("Starting Exo Driver")
        rospy.spin()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.get_joint_pos()
            rate.sleep()

def start():
    mode = rospy.get_param("mode", "test")

    if mode == "exo":
        exo_driver = ExoDriver(list(map(float, sys.argv[2].split(','))))
        exo_driver.start()
    elif mode == "test":
        test_driver = TestDriver()
        test_driver.start()

if __name__ == "__main__": 
    start()