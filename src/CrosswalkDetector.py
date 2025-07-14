#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

import rospy
import rospkg

import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from ultralytics import YOLO

import os

# model_dir = '/home/icelab/Downloads/best1.pt'

class CrosswalkDetector():
    def __init__(self, show=False):
        rospy.init_node('CrosswalkDetector')

        rospy.Subscriber("/usb_cam_down/image_raw", Image, self.usbcam_callback, queue_size=1)
        self.pub_crosswalk = rospy.Publisher('crosswalk_detected', Bool, queue_size=1)

        self.loop_hz = 10

        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])

        rp = rospkg.RosPack()
        path = os.path.join(rp.get_path('damn'), 'src', 'weights','best.pt')
        self.model = YOLO(path)

        self.crosswalk_detected = False
        self.show = show

    def usbcam_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def do(self):
        rospy.loginfo('Waiting camera--------------')
        rospy.wait_for_message("/usb_cam_down/image_raw", Image)
        rospy.loginfo("Camera Ready --------------")

        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            results = self.model(self.image, stream=False, verbose = False)
            for result in results:
                boxes = result.boxes  # Boxes object for bbox outputs

            if boxes and boxes.conf[0] > 0.7:
                x_center = int(boxes.xywh[0][0])
                y_center = int(boxes.xywh[0][1])
                
                if self.show:
                    result_img = results[0].plot()
                    result_img = cv2.circle(result_img, (x_center, y_center), 20, (255,0,0), 5)
                    cv2.imshow('crosswalk',result_img)
                    cv2.waitKey(1)

                if y_center > 300:
                    self.crosswalk_detected = True
                else:
                    self.crosswalk_detected = False
            else:
                self.crosswalk_detected = False

            self.pub_crosswalk.publish(self.crosswalk_detected)
            rate.sleep()
                
if __name__ == '__main__':
    cross = CrosswalkDetector(show=False)
    cross.do()