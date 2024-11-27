#!/usr/bin/env python3
import rospy
from pynput import keyboard
from std_msgs.msg import String

class KeystrokeListener:
    def __init__(self):
        rospy.init_node('keystroke_listener', anonymous=True)
        self.pub = rospy.Publisher('keystrokes', String, queue_size=10)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            rospy.loginfo(f'Key {key.char} pressed')
            self.pub.publish(f'Key {key.char} pressed')
        except AttributeError:
            rospy.loginfo(f'Special key {key} pressed')
            self.pub.publish(f'Special key {key} pressed')

    def on_release(self, key):
        rospy.loginfo(f'Key {key} released')
        self.pub.publish(f'Key {key} released')
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    listener = KeystrokeListener()
    listener.run()