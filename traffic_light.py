#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor
from cv_bridge import CvBridge

class GreenLightStarter:
    def __init__(self):
        rospy.init_node('green_light_starter', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.motor_pub = rospy.Publisher("/xycar_motor", xycar_motor, queue_size=1)

        self.drive_msg = xycar_motor()
        self.drive_msg.angle = 0.0
        self.drive_msg.speed = 0.0

        self.green_detected = False

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 신호등 ROI 영역 설정
        roi = frame[63:97, 392:431]

        # HSV 변환 및 초록색 필터링
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 초록불 감지 기준
        green_area = cv2.countNonZero(mask)
        if green_area > 200:
            self.green_detected = True
            rospy.loginfo("Green light detected. Starting vehicle.")

        # 차량 제어
        if self.green_detected:
            self.drive_msg.speed = 20.0  # 출발 속도
        else:
            self.drive_msg.speed = 0.0  # 정지

        self.motor_pub.publish(self.drive_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GreenLightStarter()
        node.run()
    except rospy.ROSInterruptException:
        pass
