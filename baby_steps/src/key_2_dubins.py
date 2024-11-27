#!/usr/bin/env python3

import numpy as np 
import rospy
from duckietown_msgs.msg import WheelsCmdStamped,WheelEncoderStamped
from pynput import keyboard
import time


class Key2Dubins:
    def __init__(self):
        self.pub = rospy.Publisher("/jade/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.r_tick_sub = rospy.Subscriber("/jade/right_wheel_encoder_node/tick", WheelEncoderStamped, self.r_tick_cb)
        self.r_tick = 0
        self.next_r_tick = 0
        self.l_tick_sub = rospy.Subscriber("/jade/left_wheel_encoder_node/tick", WheelEncoderStamped, self.l_tick_cb)
        self.l_tick = 0
        self.next_l_tick = 0
        self.wheels_cmd = WheelsCmdStamped()
        self.wheels_cmd.header.stamp = rospy.Time.now()
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.right_trim = 1.0
        self.left_trim = 1.1
        self.l_base =0.10
        self.speed = 0.1
        self.wheel_radius = 0.03
        self.ticks_per_rev = 135
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def r_tick_cb(self, msg):
        self.r_tick = msg.data
    def l_tick_cb(self, msg):
        self.l_tick = msg.data
    def forward(self, distance):
        d_to_tick = self.ticks_per_rev * distance/(2*np.pi*self.wheel_radius)
        self.next_r_tick = self.r_tick + d_to_tick
        self.next_l_tick = self.l_tick + d_to_tick
        v_in_rot = self.speed / self.wheel_radius
        while self.r_tick < self.next_r_tick and self.l_tick < self.next_l_tick:
            self.wheels_cmd.vel_left = self.speed* self.left_trim
            self.wheels_cmd.vel_right = self.speed* self.right_trim
            self.pub.publish(self.wheels_cmd)
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.pub.publish(self.wheels_cmd)
    def backward(self, distance):
        d_to_tick = self.ticks_per_rev * distance/(2*np.pi*self.wheel_radius)
        self.next_r_tick = self.r_tick + d_to_tick
        self.next_l_tick = self.l_tick +d_to_tick
        v_in_rot = self.speed / self.wheel_radius
        while self.r_tick < self.next_r_tick and self.l_tick < self.next_l_tick:
            self.wheels_cmd.vel_left = -self.speed* self.left_trim
            self.wheels_cmd.vel_right = -self.speed* self.right_trim
            self.pub.publish(self.wheels_cmd)
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.pub.publish(self.wheels_cmd)
    def hardcode_curve(self):
        omega = 0.3333 
        radius = 0.30
        vr = 0.5#omega * (radius + self.l_base/2)*3
        vl = 0.25 #omega * (radius - self.l_base/2)*3
        tic = time.time()
        while time.time() - tic < 10:
            self.wheels_cmd.vel_left = vl
            self.wheels_cmd.vel_right = vr
            self.pub.publish(self.wheels_cmd)
            time.sleep(0.1)
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.pub.publish(self.wheels_cmd)
    def on_spots(self):
        d_to_tick = 135
        self.next_l_tick= self.l_tick+d_to_tick
        self.next_r_tick = self.r_tick+d_to_tick
        while self.r_tick < self.next_r_tick and self.l_tick < self.next_l_tick:
            self.wheels_cmd.vel_left = -0.3* self.left_trim
            self.wheels_cmd.vel_right = 0.3* self.right_trim
            self.pub.publish(self.wheels_cmd)
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.pub.publish(self.wheels_cmd)
    def curve(self, angle,radius, type):
        rad = np.deg2rad(angle)

        omega = self.speed/radius

        if type == 'l':
            v_l = omega *(radius - self.l_base/2)
            v_r = omega *(radius + self.l_base/2)
            r_dist = (radius + self.l_base/2) * rad
            l_dist = (radius - self.l_base/2) * rad
        elif type == 'r':
            v_l = omega *(radius + self.l_base/2)
            v_r = omega *(radius - self.l_base/2)
            r_dist = (radius - self.l_base/2) * rad
            l_dist = (radius + self.l_base/2) * rad
        print('calculated speeds', v_l, v_r)
        print('calculated distances', r_dist, l_dist)
        self.next_r_tick = self.r_tick + self.ticks_per_rev*r_dist/(2*np.pi*self.wheel_radius)
        self.next_l_tick = self.l_tick + self.ticks_per_rev*l_dist/(2*np.pi*self.wheel_radius)
        finish_l = False
        while not finish_l:
            self.wheels_cmd.vel_left = v_l * self.left_trim
            self.wheels_cmd.vel_right = v_r * self.right_trim
            self.pub.publish(self.wheels_cmd)
            finish_l = self.l_tick > self.next_l_tick
            finish_r = self.r_tick > self.next_r_tick
            print('mot cmds', self.wheels_cmd.vel_left, self.wheels_cmd.vel_right)
            print('ldist', self.ticks_per_rev*l_dist/(2*np.pi*self.wheel_radius),'r_dist', self.ticks_per_rev*r_dist/(2*np.pi*self.wheel_radius))
            print('diff of r_tick  and l_tick to next tick',finish_l,finish_r, self.next_l_tick - self.l_tick, self.next_r_tick -self.r_tick)
        print('dub complete')
        self.wheels_cmd.vel_left = 0.0
        self.wheels_cmd.vel_right = 0.0
        self.pub.publish(self.wheels_cmd)
        


    def on_press(self, key):
        try:
            rospy.loginfo(f'Key {key.char} pressed')
            if key.char == 'w':
                self.forward(1.0)  # Move forward by 1 meter
            elif key.char == 'a':
                self.hardcode_curve()
            elif key.char == 'd':
                self.curve(90, 0.1, 'r')
            elif key.char == 'f':
                self.on_spots()
            elif key.char == 's':
                self.backward(0.5)  # Move backward by 1 meter
            elif key.char == 'h':
                self.hardcode_curve()            
        except AttributeError:
            rospy.loginfo(f'Special key {key} pressed')

    def on_release(self, key):
        rospy.loginfo(f'Key {key} released')
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # print('executing')
            # print('right wheel', self.r_tick, self.next_r_tick)


            rate.sleep()
if __name__ == '__main__':
    print("Starting key_2_dubins")
    rospy.init_node('key_2_dubins', anonymous=True)
    key_2_dubins = Key2Dubins()
    key_2_dubins.execute()
        