"""
Copyright (c) 2024 git-ToxiouS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

#!/usr/bin/env python3
import struct
import time
import serial
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

XAXIS = 0
YAXIS = 1
WAXIS = 2

def short2bytes(sval):
    val = struct.pack("h",sval)
    return [val[0],val[1]]

class Bot(Node):
    def __init__(self):
        super().__init__('rb5_control')
        self.serial_port = "/dev/ttyUSB0"
        self.servo_port = 60
        self.motor_ports = {
            "fl": 10,
            "fr": 1,
            "rl": 2,
            "rr": 9
        }
        self.motor_gain = {
            "fl": -1,
            "fr": 1,
            "rl": -1,
            "rr": 1
        }
        self.pen_up_angle = 130
        self.pen_down_angle = 85
        self.joy_axes = {
            XAXIS: 0,
            YAXIS: 1,
            WAXIS: 2
        }
        self.joy_dead_zone = {
            XAXIS: 0.05,
            YAXIS: 0.05,
            WAXIS: 0.05
        }
        self.joy_gain = {
            XAXIS: 64,
            YAXIS: 64,
            WAXIS: 64
        }
        # Save all buttons that are using for functions later
        self.joy_overclock_button = 4
        self.joy_highspeed_button = 1
        self.joy_midspeed_button = 0
        self.joy_lowspeed_button = 3
        self.joy_auto_button = 7
        self.joy_wiggle_button = 6
        self.joy_pen_button = 8
        self.joy_reset_orientation_button = 14
        self.ser = None

        # Subcribe all functions that need access to the joy node
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.move_joy, 1)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.gain_control, 1)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.auto_control, 1)
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.wiggle, 1)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.update_orientation, 1)
        
        self.last_time = 0
        self.orientation = 0

    def start(self):
        self.ser = serial.Serial(self.serial_port, 115200, timeout=10)
        time.sleep(1)
    def stop(self):
        self.set_motors([0,0,0,0])
        self.pen_up()
        self.ser.close()
    def set_servo(self, angle):
        if 0 <= angle < 180:
            self.ser.write(bytearray([0xff, 0x55, 5, 0, 0x2, 0x21, self.servo_port, angle]))
            time.sleep(0.01)
            self.ser.flush()
    def set_motor(self, motor_id, speed):
        set_speed = int(speed * self.motor_gain[motor_id])
        self.ser.write(bytearray([0xff,0x55,0x6,0x0,0x2,0xa,self.motor_ports[motor_id]]+short2bytes(set_speed)))
        time.sleep(0.01)
    def set_motors(self, speeds):
        self.set_motor("fl", speeds[0])
        self.set_motor("fr", speeds[1])
        self.set_motor("rl", speeds[2])
        self.set_motor("rr", speeds[3])
        self.ser.flush()
    def move_joy(self, joy_cmd):
        x_joy = joy_cmd.axes[self.joy_axes[XAXIS]]
        y_joy = joy_cmd.axes[self.joy_axes[YAXIS]]
        w_joy = joy_cmd.axes[self.joy_axes[WAXIS]]
        x = (x_joy * self.joy_gain[XAXIS]) if abs(x_joy) > self.joy_dead_zone[XAXIS] else 0
        y = (y_joy * self.joy_gain[YAXIS]) if abs(y_joy) > self.joy_dead_zone[YAXIS] else 0
        w = (w_joy * self.joy_gain[WAXIS]) if abs(w_joy) > self.joy_dead_zone[WAXIS] else 0

        orientation_rad = self.orientation * math.pi / 180
        x_rot = x * math.cos(orientation_rad) + y * math.sin(-orientation_rad)
        y_rot = x * math.sin(orientation_rad) + y * math.cos(orientation_rad)
        x = x_rot
        y = y_rot

        fl = -x + y - w
        fr =  x + y + w
        rl =  x + y - w
        rr = -x + y + w
        self.set_motors([fl, fr, rl, rr])

        if joy_cmd.buttons[self.joy_pen_button]:
            self.pen_down()
        else:
            self.pen_up()

        if joy_cmd.buttons[self.joy_reset_orientation_button]:
            self.orientation = 0

    def move_cmd(self, x_joy, y_joy, w_joy):
        # A function to be able to put normalised speed values into to drive at a specific speed
        x = (x_joy * self.joy_gain[XAXIS]) if abs(x_joy) > self.joy_dead_zone[XAXIS] else 0
        y = (y_joy * self.joy_gain[YAXIS]) if abs(y_joy) > self.joy_dead_zone[YAXIS] else 0
        w = (w_joy * self.joy_gain[WAXIS]) if abs(w_joy) > self.joy_dead_zone[WAXIS] else 0

        orientation_rad = self.orientation * math.pi / 180
        x_rot = x * math.cos(orientation_rad) + y * math.sin(-orientation_rad)
        y_rot = x * math.sin(orientation_rad) + y * math.cos(orientation_rad)
        x = x_rot
        y = y_rot

        fl = -x + y - w
        fr =  x + y + w
        rl =  x + y - w
        rr = -x + y + w
        self.set_motors([fl, fr, rl, rr])

    def auto_control(self, joy_cmd):
        # Drive at "1" speed straight ahead for a specific amount of time, when a button is pressed
        if joy_cmd.buttons[self.joy_auto_button]:
            self.get_logger().info("Auto")
            x_joy = 0
            y_joy = 1
            w_joy = 0
            Bot.move_cmd(self, x_joy, y_joy, w_joy)

            time.sleep(2.05)
            Bot.move_joy(self, joy_cmd)
    
    def wiggle(self, joy_cmd):
        # Get the robot to "wiggle", full speed in one direction, full speed to the other
        if joy_cmd.buttons[self.joy_wiggle_button]:
            self.get_logger().info("Wiggle!")
            x_joy = -1
            y_joy = -1
            w_joy = -1
            Bot.move_cmd(self, x_joy, y_joy, w_joy)

            time.sleep(0.1)

            x_joy = 1
            y_joy = 1
            w_joy = 1
            Bot.move_cmd(self, x_joy, y_joy, w_joy)

            time.sleep(0.1)
            Bot.move_joy(self, joy_cmd)


    def gain_control(self, joy_cmd):
        # Four buttons are mapped to change to motor_gain to specific values.
        # A message is printed to clarify what setting the robot is in
        # Putting the gain any higher than 2 is not recommended (it might break itself)

        if joy_cmd.buttons[self.joy_overclock_button]:
            self.get_logger().info("Overclocked!")
            self.motor_gain = {
               "fl": -2,
               "fr": 2,
               "rl": -2,
               "rr": 2
               }
        elif joy_cmd.buttons[self.joy_highspeed_button]:
            self.get_logger().info("High speed")
            self.motor_gain = {
               "fl": -1.5,
               "fr": 1.5,
               "rl": -1.5,
               "rr": 1.5
               }
        elif joy_cmd.buttons[self.joy_midspeed_button]:
            self.get_logger().info("Normal speed")
            self.motor_gain = {
               "fl": -1,
               "fr": 1,
               "rl": -1,
               "rr": 1
               }
        elif joy_cmd.buttons[self.joy_lowspeed_button]:
            self.get_logger().info("Snailspeed")
            self.motor_gain = {
               "fl": -0.75,
               "fr": 0.75,
               "rl": -0.75,
               "rr": 0.75
               }
            
    def update_orientation(self, imu_data):
        s = imu_data.header.stamp.sec
        ns = imu_data.header.stamp.nanosec
        total_ns = s * 1000000000 + ns
        if self.last_time == 0:
            self.last_time = total_ns
            return
        diff_ns = total_ns - self.last_time
        self.last_time = total_ns

        z = imu_data.angular_velocity.z * 180 / math.pi
        self.orientation = self.orientation + (z * diff_ns / 1000000000)
        while self.orientation > 180:
            self.orientation = self.orientation - 360
        while self.orientation < -180:
            self.orientation = self.orientation + 360
    def pen_up(self):
        self.set_servo(self.pen_up_angle)
    def pen_down(self):
        self.set_servo(self.pen_down_angle)

def main(args=None):
    rclpy.init(args=args)
    bot = Bot()
    bot.start()
    print("init done")

    rclpy.spin(bot)
    bot.stop()
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
