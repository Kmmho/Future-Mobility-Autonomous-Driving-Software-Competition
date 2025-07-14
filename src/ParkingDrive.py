#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

from icelab_msgs.msg import icelab_ultrasounds
from icelab_msgs.msg import icelab_motor

import rospy
import time

class ParkingDrive():
    def __init__(self, hz=100) -> None:
        rospy.init_node('ParkingDrive')        
        rospy.Subscriber("/sonics", icelab_ultrasounds, self.cb_sonic, queue_size=1)
        self.motor = rospy.Publisher('/motor_sub', icelab_motor, queue_size=1) # 모터 토픽을 발행할 것임을 선언
        self.motor_msg = icelab_motor()

        self.loop_hz = hz
        self.sonic = []

    def cb_sonic(self, msg: icelab_ultrasounds) -> None:
        self.sonic = list(msg.ranges[i].range for i in range(6))

    def drive(self, angle: float, speed: float) -> None:        
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor.publish(self.motor_msg)
    
    def parking_process_1(self):
        for i in range(10):
            self.drive(0,0)
            rospy.sleep(0.1)
        for i in range(60):
            self.drive(-18,80)
            rospy.sleep(0.1)
        for i in range(60):
            self.drive(18,-80)
            rospy.sleep(0.1)
        for i in range(40):
            self.drive(-18,80)
            rospy.sleep(0.1)
        for i in range(80):
            self.drive(0,-80)
            rospy.sleep(0.1)
        for i in range(20):
            self.drive(0,0)
            rospy.sleep(0.1)

    def parking_process_2(self):
        for i in range(20):
            self.drive(0,0)
            rospy.sleep(0.1)
        for i in range(20):
            self.drive(0,80)
            rospy.sleep(0.1)
        for i in range(100):
            self.drive(-18,100)
            rospy.sleep(0.1)
        for i in range(40):
            self.drive(18,-100)
            rospy.sleep(0.1)
        for i in range(50):
            self.drvie(0,100)
            rospy.sleep(0.1)



    def main_loop(self):
        rospy.loginfo('Waiting -------------------')
        rospy.wait_for_message("/sonics", icelab_ultrasounds)
        rospy.loginfo("Sonic Ready --------------")

        start_point = False
        start_time = time.time()
        finish_flag = False

        self.drive(0,100)

        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            if not finish_flag:
                if (time.time()-start_time > 5) and self.sonic[3]<900 and not start_point:
                    rospy.loginfo('start_point')
                    start_point = True
                if start_point and self.sonic[2] < 900:
                    rospy.loginfo('start parking')
                    self.parking_process_1()
                    rospy.loginfo('parking finish')
                    self.parking_process_2()
                    self.drive(0,0)
                    finish_flag = True

            rate.sleep()

if __name__ == '__main__':    
    parking = ParkingDrive()
    parking.main_loop()
    
        