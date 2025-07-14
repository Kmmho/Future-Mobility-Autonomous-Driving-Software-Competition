#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
import cv2

from sensor_msgs.msg import LaserScan
from typing import Tuple
from std_msgs.msg import Bool
import rospy

class ObstacleDetector():
    def __init__(self,
                 range_min: float = 0.3,
                 range_max: float = 1.6,
                 width: int = 256,
                 height: int = 256,
                 threshold: float = 0.3) -> None:        
        self.range_min = range_min
        self.range_max = range_max
        self.width, self.height = width,height
        self.half_width, self.half_height = self.width //2, self.height //2

        rospy.init_node('ObsDetector')
        self.pub_obstacle = rospy.Publisher('obstacle_detected', Bool, queue_size=1)
        rospy.Subscriber("/scan"                    , LaserScan         , self.cb_scan          , queue_size=1)
        
        self.threshold = threshold


    def cb_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def main(self):
        rospy.wait_for_message("/scan", LaserScan)
        rospy.loginfo("LiDAR Ready  --------------")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.do(self.scan_msg)
            rate.sleep()


    def do(self, scan_msg: LaserScan, debug: bool = False) -> Tuple[bool, float, float, float, np.ndarray]:
        # initialize
        temp_img = np.zeros((self.width, self.height), dtype=np.uint8)        
        intensity_max = max(scan_msg.intensities)

        # filtering out-of-range values
        ranges = np.array(scan_msg.ranges)
        ranges[np.where((ranges<self.range_min) | (ranges>self.range_max))] = 0

        # creating top-down view image
        for idx, (rg, it) in enumerate(zip(ranges, scan_msg.intensities)):
            rg = rg / self.range_max * self.half_width # converting the unit of range values from meters to pixels
            curr_angle = scan_msg.angle_min + idx*scan_msg.angle_increment

            x, y = (self.half_width+int(np.cos(curr_angle)*rg),
                    self.half_height+int(np.sin(curr_angle)*rg))
            if curr_angle>3*np.pi/4 or curr_angle<-3*np.pi/4:
                temp_img[x, y] = int(it/intensity_max*255)

        # finding center of left and right corns            
        notzero_x_index = np.array(np.where(np.sum(temp_img[:self.half_height,:],axis=0)!=0)) # summing up each column and finding which column sum is not zero
        
        # creating debug image
        img_bgr = cv2.cvtColor(temp_img, cv2.COLOR_GRAY2BGR)

        if len(notzero_x_index[0]) <= self.half_width*self.threshold:
            # return False, img_bgr, None
            self.pub_obstacle.publish(False)
            return

        center_x_px = int(np.mean(notzero_x_index))
        center_x_cm = (center_x_px-self.half_width)*self.range_max/self.half_width

        if debug:
            img_bgr[self.half_height,notzero_x_index,:]=(255,0,255)
            img_bgr = cv2.circle(img_bgr, (center_x_px,self.half_height), 5, (255,0,255), 5)
        # return True, img_bgr, center_x_cm
        self.pub_obstacle.publish(True)
    


if __name__ == '__main__':
    OD = ObstacleDetector()
    OD.main()