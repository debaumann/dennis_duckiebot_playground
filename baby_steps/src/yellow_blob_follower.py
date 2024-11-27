#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import WheelsCmdStamped
#/code/catkin_ws
class YellowBlobFever:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/jade/camera_node/image/compressed", CompressedImage, self.callback)
        self.pub = rospy.Publisher("/jade/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.images = []
        self.im_w = 640
        self.im_h = 480
        
    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.images.append(cv_image)

    def blobber(self, im):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        # define range of yellow color in HSV
        lower_yellow = np.array([20, 50, 50])
        upper_yellow = np.array([50, 255, 255])
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Bitwise-AND mask and original image
        result = cv2.bitwise_and(im, im, mask=mask)
        gray_result = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)

        # Apply a binary threshold to get a binary mask
        _, binary_mask = cv2.threshold(gray_result, 1, 255, cv2.THRESH_BINARY)
        binary_mask_np = np.array(binary_mask)

        # BEGIN: Apply morphological operations to remove small noise
        kernel = np.ones((5, 5), np.uint8)
        cleaned_mask = cv2.morphologyEx(binary_mask_np, cv2.MORPH_OPEN, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # Discard contours with small area
        min_contour_area = 100  # You can adjust this threshold
        contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
        # Get bounding boxes for each contour
        bounding_boxes = [cv2.boundingRect(contour) for contour in contours]
        total_bbox_area = np.sum([w*h for (x, y, w, h) in bounding_boxes])

        bbox_w_frac = []
        for (x, y, w, h) in bounding_boxes:
            bbox_w_frac.append((np.array([x, y, w, h, w*h/total_bbox_area])))

        bbox_w_frac = sorted(bbox_w_frac, key=lambda x: x[4])
        print(bbox_w_frac)
        
        return bbox_w_frac



    def calculate_wheels(self, bbox_w_frac):
        if bbox_w_frac:
            x, y, w, h, f = bbox_w_frac[-1]
            c_X = x + w / 2
            c_Y = y + h / 2
            l = c_X/self.im_w
            r = (self.im_w - c_X)/self.im_w
            kp = 0.5 
            vel = 0.2
            return l*kp* vel, r*kp * vel
        else:
            return 0.0, 0.0

    def run(self):
        rate = rospy.Rate(10)
        print("Waiting for image")
        rospy.wait_for_message("/jade/camera_node/image/compressed", CompressedImage)
        while not rospy.is_shutdown():
            if self.images:
                im = self.images[-1]
                self.images = []
                bbox_w_frac = self.blobber(im)
                l, r = self.calculate_wheels(bbox_w_frac)
                msg = WheelsCmdStamped()
                msg.header.stamp = rospy.Time.now()
                msg.vel_left = l
                msg.vel_right = r
                print(f"Left: {l}, Right: {r}")
                self.pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    controller = YellowBlobFever()
    rospy.init_node('yellow_blob_follower')
    controller.run()