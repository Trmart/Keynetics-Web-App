"""
FILE:       aquarium/src/fredpi/fredpi/interface.py
AUTHORS:    Zach Preston, Taylor Martin
CREATED:    2023-01-20
MODIFIED:   2023-02-03
PURPOSE:    University of Idaho Senior Engineering Capstone Design Project
PROJECT:    Keynetics Key Assembler
INFO:       This is the Fred Programming Interface (FredPI) for the Fred MyCobot robot arm.
            It is a Wrapper Library for the ROS2 Library.
            It is used to simplify user control of the Fred MyCobot robot arm.
"""



# Imports.
import math
import rclpy
import time
from .JointPositionsPub import JointPositionsPub
from .OutputPinPub import OutputPinPub
from .GetJointsSub import GetJointsSub
from .GetOutputsSub import GetOutputsSub



# Automatically instantiate nodes.
rclpy.init()
joint_positions_pub = JointPositionsPub()
output_pin_pub = OutputPinPub()
get_joints_sub = GetJointsSub()
get_outputs_sub = GetOutputsSub()



# Test out by copying wave program.
def test_wave():
    # Send a stream of messages to get ROS 2 to recognize the connection.
    # Also resets the robot's position.
    print('Initializing')
    for i in range(10):
        joint_positions_pub.publish([0.0 for i in range(6)], 80)
        time.sleep(0.1)

    # Turn on the last output pin.
    print('Turning on pin')
    output_pin_pub.publish(6, True)
    time.sleep(0.5)

    # Loop 3 times.
    for i in range(3):
        # Send waving messages.
        print(f"Wave {i + 1}")
        joint_positions_pub.publish([0.0, 0.0, 0.0, math.pi / 4.0, 0.0, 0.0], 80)
        time.sleep(1.3)
        joint_positions_pub.publish([0.0, 0.0, 0.0, math.pi / -4.0, 0.0, 0.0], 80)
        time.sleep(1.3)

    # Go back to starting position.
    print('Resetting')
    joint_positions_pub.publish([0.0 for i in range(6)], 80)
    time.sleep(0.5)

    # Turn off the last output pin.
    print('Turning off pin')
    output_pin_pub.publish(6, False)
    time.sleep(0.5)



# Test out by turning on the pin for as short as possible.
def test_pulse():
    # Send a stream of messages to get ROS 2 to recognize the connection.
    # Also resets the robot's position.
    print('Initializing')
    for i in range(10):
        joint_positions_pub.publish([0.0 for i in range(6)], 80)
        time.sleep(0.1)

    # Pulse the pump output pin.
    print('Pulsing pump')
    output_pin_pub.publish(1, True, 0)






def set_coordinates():
   """Set the coordinates of the robot arm"""

def get_current_position():
   """Get the current position of the robot arm"""

def reset_position():
   """Reset the robot arm to its default position"""

def toggle_pump_power():
   """Turn the pump on or off"""

def toggle_light_power():
   """Turn the UV light on or off"""

def convert_coordiants_to_radians(coordinates):
   """Convert coordinates to radians"""
