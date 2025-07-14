#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

#=== DAMN means ===================
# Dku Autonomous Mobility eNigneers
#==================================

from sensor_msgs.msg import Image, LaserScan, Joy
from icelab_msgs.msg import icelab_ultrasounds
from icelab_msgs.msg import icelab_motor
from std_msgs.msg import Bool, Int16

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np

import LaneDetector
import LaneDetectorSliding

from ObstacleDetector import ObstacleDetector

from Kalman import KalmanPos2Vel
from PID import PID

class AutonomousDriver():
    DRIVING_MODE_STOP    = -2
    DRIVING_MODE_MANUAL  = -1
    DRIVING_MODE_LANE    =  0
    DRIVING_MODE_DETOUR  =  1

    def __init__(self) -> None:
        rospy.init_node('DAMN')

        rospy.Subscriber("/usb_cam_down/image_raw"  , Image             , self.cb_cam           , queue_size=1)
        # rospy.Subscriber("/scan"                    , LaserScan         , self.cb_scan          , queue_size=1)
        rospy.Subscriber("/joy"                     , Joy               , self.cb_joy           , queue_size=1)
        rospy.Subscriber("/crosswalk_detected"      , Bool              , self.cb_crosswalk     , queue_size=1)
        rospy.Subscriber("/traffic_signal_detected" , Int16             , self.cb_trafficsignal , queue_size=1)
        rospy.Subscriber("/obstacle_detected"       , Bool              , self.cb_obstacle      , queue_size=1)

        self.motor = rospy.Publisher('/motor_sub', icelab_motor, queue_size=1) # 모터 토픽을 발행할 것임을 선언
        self.motor_msg = icelab_motor()
        self.lane_roi_img_pub = rospy.Publisher('/roi_img', Image, queue_size=1)
        self.lane_edge_img_pub = rospy.Publisher('/edge_img', Image, queue_size=1)

        self.loop_hz = 30

        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])            # image data for Lane driving
        self.scan_msg = LaserScan()                 # scan  data for Evasion driving

        ## Lane driving 관련
        camera_width, camera_height = 1280, 720
        self.ref_center = 630
        self.kf_lane = KalmanPos2Vel(x0=np.array([camera_width//2,0]), 
                                     P0=5*np.eye(2), 
                                     q1=0.1, 
                                     q2=1, 
                                     r=50, 
                                     dt=1/self.loop_hz)
        self.pid_lane = PID(Kp=0.13, 
                            Ki=0.0, 
                            Kd=0.001, 
                            dt=1/self.loop_hz, 
                            max_u=18)
        
        self.speed_lane_found = 170
        self.speed_lane_not_found = 100

        ## Hough transform-based
        self.ld = LaneDetector.LaneDetector(width = camera_width,
                                            height = camera_height,  
                                            canny_th1 = 30,
                                            canny_th2 = 60,
                                            roi_start_row = 150,
                                            roi_end_row = 650,
                                            roi_start_column = 0,
                                            roi_end_column = 1280,
                                            margin = 300,
                                            lane_interval = 900)

        ## Sliding window-based
        self.ld_sliding = LaneDetectorSliding.LaneDetector(img_shape = (camera_width, camera_height),
                                                           lane_width=389)
        
        ## obstacle detector 관련
        # self.od = ObstacleDetector(range_max=1.6)

        self.driving_mode = AutonomousDriver.DRIVING_MODE_LANE
        self.prev_driving_mode = self.driving_mode

        self.manual_angle, self.manual_speed = 0, 0

        self.ret_od = False
        self.prev_crosswalk_detected = False

    def cb_cam(self, msg: Image) -> None:
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2.imshow('im',self.image)
        # cv2.waitKey(1)
    
    # def cb_scan(self, msg: LaserScan) -> None:
    #     self.scan_msg = msg

    def cb_sonic(self, msg: icelab_ultrasounds) -> None:
        self.sonic = msg

    def cb_crosswalk(self, msg: Bool) -> None:
        if msg.data==True and self.prev_crosswalk_detected==False:
            self.prev_crosswalk_detected=True
            self.driving_mode = AutonomousDriver.DRIVING_MODE_STOP
            rospy.logerr("STOP!!!")

    def cb_trafficsignal(self, msg: Int16) -> None:
        if msg.data==1 and self.driving_mode == AutonomousDriver.DRIVING_MODE_LANE:
            self.driving_mode = AutonomousDriver.DRIVING_MODE_STOP
            rospy.logerr("STOP!!!")
        if msg.data==2 and self.driving_mode == AutonomousDriver.DRIVING_MODE_STOP:
            self.driving_mode = AutonomousDriver.DRIVING_MODE_LANE
            self.prev_crosswalk_detected=False
            rospy.logerr("GO!!!")

    def cb_obstacle(self, msg: Bool) -> None:
        self.ret_od = msg.data

    def cb_joy(self, msg: Joy) -> None:
        if msg.buttons[0] and self.driving_mode != AutonomousDriver.DRIVING_MODE_MANUAL:
            self.prev_driving_mode = self.driving_mode
            self.driving_mode = AutonomousDriver.DRIVING_MODE_MANUAL
        elif msg.buttons[1]:
            self.driving_mode = self.prev_driving_mode

        self.manual_angle = msg.axes[3]*(-40.0)
        self.manual_speed = (msg.axes[1]*3+2) if msg.axes[1] > 0 else (msg.axes[1]*3-2)

    def drive(self, angle: float, speed: float) -> None:        
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        # rospy.loginfo(f'{angle}, {speed}')
        self.motor.publish(self.motor_msg)
    
    def lane_drive(self, debug=False):
        ret, roi_img, edge_img, x_left, x_right, x_center = self.ld.do(self.image, debug)
        if ret==LaneDetector.LaneDetector.FOUND_LANES:
            self.xh_center, _ = self.kf_lane.update(x_center)

        u = -self.pid_lane.do(self.ref_center, self.xh_center[0], self.xh_center[1])
        # rospy.loginfo(f'x={self.xh_center[0]:0.1f}, r={self.ref_center}, u={int(u)}, {x_right-x_left}')
        self.drive(int(u), self.speed_lane_found if ret==LaneDetector.LaneDetector.FOUND_LANES else self.speed_lane_not_found)
        # self.drive(int(u), 0)

        if debug:
            cv2.rectangle(roi_img, (x_left-5,self.ld.L_ROW-5), (x_left+5,self.ld.L_ROW+5), (0,255,0), 4)
            cv2.rectangle(roi_img, (x_right-5,self.ld.L_ROW-5), (x_right+5,self.ld.L_ROW+5), (255,0,255), 4)        
            cv2.rectangle(roi_img, (int(self.xh_center[0])-5,self.ld.L_ROW-5), (int(self.xh_center[0])+5,self.ld.L_ROW+5), (255,0,0), 4)
            roi_img = self.bridge.cv2_to_imgmsg(roi_img, encoding='bgr8')
            edge_img = self.bridge.cv2_to_imgmsg(edge_img)
            self.lane_roi_img_pub.publish(roi_img)
            self.lane_edge_img_pub.publish(edge_img)
    
    def lane_drive_sliding(self, debug=False):
        ret, out_img, out_polyfit_img, output = self.ld_sliding.do(self.image, debug=True)

        found = False
        if ret <= LaneDetectorSliding.LaneDetector.SUCCESS_POLY_FIT_ONE:
            found = True
            self.xh_center, _ = self.kf_lane.update(output[1])

        u = -self.pid_lane.do(self.ref_center, self.xh_center[0], self.xh_center[1])
        # rospy.loginfo(f'x={self.xh_center[0]:0.1f}, r={self.ref_center}, u={int(u)}, {output[2]-output[0]}')
        self.drive(int(u), self.speed_lane_found if found else self.speed_lane_not_found)
        # self.drive(int(u), 0)

        if debug:
            out_img = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
            out_polyfit_img = self.bridge.cv2_to_imgmsg(out_polyfit_img, encoding='bgr8')
            self.lane_roi_img_pub.publish(out_img)
            self.lane_edge_img_pub.publish(out_polyfit_img)
            
    def stop(self) -> None:
        self.drive(0, 0)

    def detour(self, count):
        if count == 0:
            rospy.logerr("First detour")
            count = 1

            ##여기부터
            # 첫 번째 회피 기동
            for i in range(50):
                self.drive(-18,100)
                rospy.sleep(0.1)  
            
            for i in range(50):
                self.drive(18,100)
                rospy.sleep(0.1)

            for i in range(1):
                self.drive(0,60)
                rospy.sleep(1)
            ##여기까지


            while True:
                # ret_od, topdown_img, center_x_cm = self.od.do(self.scan_msg, debug=True)
                # cv2.imshow('lidar', topdown_img)
                # cv2.waitKey(1)                
                self.drive(0,60)
                rospy.sleep(0.1)
                if self.ret_od:
                    break
            
            rospy.logerr("second detour")

            # 두 번째 회피 기동
            for i in range(48):
                self.drive(18,100)
                rospy.sleep(0.1)  
            
            for i in range(38):
                self.drive(-18,100)
                rospy.sleep(0.1)

            self.drive(0,20)
            rospy.sleep(1)
            self.driving_mode=AutonomousDriver.DRIVING_MODE_LANE

        return count
                
    def main_loop(self):
        rospy.loginfo('Waiting -------------------')
        rospy.wait_for_message("/usb_cam_down/image_raw", Image)
        rospy.loginfo("Camera Ready --------------")

        self.xh_center=np.array([0,0])
        self.driving_mode = AutonomousDriver.DRIVING_MODE_LANE
        rate = rospy.Rate(self.loop_hz)

        od_count = 0

        mode = 1  # 모드가 1이면 장애물 피한다...

        while not rospy.is_shutdown():
            try:
                if mode:
                    # ret_od, topdown_img, center_x_cm = self.od.do(self.scan_msg, debug=True)
                    # cv2.imshow('lidar', topdown_img)
                    # cv2.waitKey(1)
                    if self.ret_od:
                        if od_count == 0:
                            self.driving_mode = AutonomousDriver.DRIVING_MODE_DETOUR
                        od_count = self.detour(od_count)
                
                if self.driving_mode == AutonomousDriver.DRIVING_MODE_LANE:
                    # self.lane_drive(debug=True)
                    self.lane_drive_sliding(debug=True)
                elif self.driving_mode == AutonomousDriver.DRIVING_MODE_STOP:
                    self.stop()
                elif self.driving_mode == AutonomousDriver.DRIVING_MODE_DETOUR:
                    self.drive(0,30)
                rate.sleep()

            except KeyboardInterrupt:
                self.stop()
                break

if __name__ == '__main__':    
    ads = AutonomousDriver()
    ads.main_loop()