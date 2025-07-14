#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge

class TrafficSignalDetector():
    NORMAL = 0
    RED = 1
    GREEN = 2

    def __init__(self, show = False) -> None:
        rospy.init_node('TrafficSignalDetector')

        rospy.Subscriber("/usb_cam_up/image_raw", Image, self.usbcam_callback, queue_size=1)

        self.image = np.empty(shape=[0])

        self.pub_traffic_signal = rospy.Publisher('traffic_signal_detected', Int16, queue_size=1)
        self.bridge = CvBridge()
        self.show = show

        self.min_green_blob_area = 3500
        self.min_red_blob_area = 8700
                
    def usbcam_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

    def detect_traffic_signal(self):        
        roi_image = self.image[0:200, :]

        img_hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(img_hsv, (150, 100, 0), (210, 255, 255))
        green_mask = cv2.inRange(img_hsv, (60, 70, 100), (95, 255, 255))
        # cv2.imshow('mask2', green_mask)
        
        kernel = np.ones((7,7), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_num = cv2.countNonZero(red_mask)

        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN,kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_num = cv2.countNonZero(green_mask)

        connectivity = 8
        _, _, red_stats, _ = cv2.connectedComponentsWithStats(red_mask, connectivity, cv2.CV_32S)
        _, _, green_stats, _ = cv2.connectedComponentsWithStats(green_mask, connectivity, cv2.CV_32S)
        
        red_blobs = [i for i, stat in enumerate(red_stats) if stat[4] >= self.min_red_blob_area]
        green_blobs = [i for i, stat in enumerate(green_stats) if stat[4] >= self.min_green_blob_area]

        if self.show:
            img_result_with_blobs = roi_image.copy()

            for blob_id in red_blobs:
                if blob_id == 0:
                    continue
                x, y, w, h, area = red_stats[blob_id]
                cv2.rectangle(img_result_with_blobs, (x, y), (x+w, y+h), (0,0,255),2)    

            for blob_id in green_blobs:
                if blob_id == 0:
                    continue
                x, y, w, h, area = green_stats[blob_id]
                cv2.rectangle(img_result_with_blobs, (x, y), (x+w, y+h), (0,255,0),2)    
            cv2.imshow('dd', red_mask)
            cv2.imshow('img_result_with_blobs', img_result_with_blobs)
            cv2.waitKey(1)
        #print(red_num)
        if green_num > self.min_green_blob_area:
            state = TrafficSignalDetector.GREEN
        elif red_num > self.min_red_blob_area:
            state = TrafficSignalDetector.RED
        else:
            state = TrafficSignalDetector.NORMAL

        return state
    
    def do(self):
        rospy.loginfo('Waiting camera--------------')
        rospy.wait_for_message("/usb_cam_up/image_raw", Image)
        rospy.loginfo("Camera Ready2 --------------")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            state = self.detect_traffic_signal()
            self.pub_traffic_signal.publish(state)
            if state == TrafficSignalDetector.GREEN:
                rospy.loginfo('traffic: green light' )
            elif state == TrafficSignalDetector.RED:
                rospy.loginfo('traffic: red light')
            rate.sleep()
    
if __name__ == '__main__':
    TSD = TrafficSignalDetector(show=True)
    TSD.do()