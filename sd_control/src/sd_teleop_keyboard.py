#!/usr/bin/env python3

from __future__ import print_function

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sd_msgs.msg import SDControl

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to SD_Control!
Uses "w, a, s, d, x" keys or numpad
---------------------------
Apply throttle:
   'w' or '8'
Ease off throttle:
    's' or '5'
Turn steering left:
    'a' or '4'
Turn steering right:
    'd' or '6'
 Apply brakes:
    'x' or '2'
    
CTRL-C to quit

Notes:
The twizy has a deadband of throttle, and requires >25% throttle to begin moving.
Steering will centre upon braking
All requests ramp upon sustain key press. Depress key to maintain steady request
"""

throttleKeys = {
    'w': (1.0, 0.0),
    '8': (1.0, 0.0)
}

overrunKeys = {
    's': (1.0, 0.0),
    '5': (1.0, 0.0)
}

brakeKeys = {
    'x': (5.0, 0.0),
    '2': (5.0, 0.0)
}

leftKeys = {
    'a': (4.0, 0.0),
    '4': (4.0, 0.0)
}

rightKeys = {
    'd': (4.0, 0.0),
    '6': (4.0, 0.0)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.pub = self.create_publisher(SDControl, '/sd_control', 10)
        self.control_msg = SDControl()
        self.overrun = 0.0
        self.brake = 0.0
        self.torque = 0.0
        self.steer = 0.0

    def run(self):
        print(msg)
        try:
            while True:
                key = getKey()
                if key in throttleKeys.keys():
                    self.torque += throttleKeys[key][0]
                    self.brake = 0.0
                elif key in overrunKeys.keys():
                    self.torque -= overrunKeys[key][0]
                    self.brake = 0.0
                elif key in brakeKeys.keys():
                    self.brake -= brakeKeys[key][0]
                    self.steer = 0.0
                elif key in leftKeys.keys():
                    self.steer += leftKeys[key][0]
                elif key in rightKeys.keys():
                    self.steer -= rightKeys[key][0]
                else:
                    self.brake = 0.0
                    if key == '\x03':
                        break
                
                self.torque = min(100.0, self.torque)
                self.torque = max(0.0, self.torque)
                self.steer = max(-100.0, self.steer)
                self.steer = min(100.0, self.steer)
                self.brake = max(-100.0, self.brake)

                if self.brake:
                    self.control_msg.torque = self.brake
                else:
                    self.control_msg.torque = self.torque
                self.control_msg.steer = self.steer
                print("Throttle ", self.torque, " brake ", self.brake, " steer ", self.steer)
                self.pub.publish(self.control_msg)

        except Exception as e:
            print(e)
        finally:
            self.control_msg.torque = 0.0
            self.control_msg.steer = 0.0
            self.pub.publish(self.control_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
