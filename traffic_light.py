#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, rospy
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge

# ROI 좌표 (y1, y2, x1, x2)
RED_ROI    = (62, 91, 206, 242)
YELLOW_ROI = (61, 94, 298, 338)
GREEN_ROI  = (63, 97, 392, 431)

# HSV 마스크 범위
RED_LOWER1, RED_UPPER1   = np.array([0, 100, 100]),   np.array([10, 255, 255])
RED_LOWER2, RED_UPPER2   = np.array([160, 100, 100]), np.array([179, 255, 255])
YELLOW_LOWER, YELLOW_UPPER = np.array([20, 100, 100]), np.array([40, 255, 255])
GREEN_LOWER, GREEN_UPPER   = np.array([40, 100, 100]), np.array([80, 255, 255])

PIXEL_THRESHOLD = 50      # 픽셀 수 임계치
GO_SPEED        = 800     # 출발 속도 (PWM 스케일)
STOP_SPEED      = 0

class TrafficLightController:
    def __init__(self):
        rospy.init_node('traffic_light_controller', anonymous=True)
        self.bridge     = CvBridge()
        self.pub        = rospy.Publisher("/xycar_motor", XycarMotor, queue_size=1)
        self.sub        = rospy.Subscriber("/usb_cam/image_raw", Image, self.cb_image)
        self.cmd        = XycarMotor()
        self.cmd.angle  = 0

        self.started    = False
        self.last_stamp = rospy.Time(0)

    def cb_image(self, msg: Image):
        # 1) 시뮬/토픽 재시작 감지
        if msg.header.stamp < self.last_stamp:
            self.started = False
            rospy.loginfo("🔄 Reset detected → STOPPED")
        self.last_stamp = msg.header.stamp

        # 2) 이미지 변환
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 3) 각 색 ROI별로 픽셀 수 계산
        # 빨강
        r1, r2, c1, c2 = RED_ROI
        hsv_r    = cv2.cvtColor(frame[r1:r2, c1:c2], cv2.COLOR_BGR2HSV)
        mask_r   = cv2.inRange(hsv_r, RED_LOWER1, RED_UPPER1) | cv2.inRange(hsv_r, RED_LOWER2, RED_UPPER2)
        red_area = cv2.countNonZero(mask_r)

        # 노랑
        r1, r2, c1, c2 = YELLOW_ROI
        hsv_y      = cv2.cvtColor(frame[r1:r2, c1:c2], cv2.COLOR_BGR2HSV)
        mask_y     = cv2.inRange(hsv_y, YELLOW_LOWER, YELLOW_UPPER)
        yellow_area = cv2.countNonZero(mask_y)

        # 초록
        r1, r2, c1, c2 = GREEN_ROI
        hsv_g      = cv2.cvtColor(frame[r1:r2, c1:c2], cv2.COLOR_BGR2HSV)
        mask_g     = cv2.inRange(hsv_g, GREEN_LOWER, GREEN_UPPER)
        green_area = cv2.countNonZero(mask_g)

        rospy.loginfo_throttle(1, f"[AREA] R={red_area}  Y={yellow_area}  G={green_area}")

        # 4) 이미 출발한 상태에서 빨간불 감지 시 플래그 리셋
        if self.started and red_area > PIXEL_THRESHOLD:
            self.started = False
            rospy.loginfo("✋ Red detected after start → RESET to STOPPED")

        # 5) 출발 전(started=False)일 때만 신호등 판별
        if not self.started:
            if green_area > PIXEL_THRESHOLD:
                self.started = True
                self.cmd.speed = GO_SPEED
                rospy.loginfo("✅ Green detected → START")
            else:
                # 빨강·노랑·없음 모두 정지
                self.cmd.speed = STOP_SPEED
        else:
            # 6) 출발 이후에는 계속 직진
            self.cmd.speed = GO_SPEED

        # 7) 퍼블리시
        self.pub.publish(self.cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        TrafficLightController().run()
    except rospy.ROSInterruptException:
        pass
