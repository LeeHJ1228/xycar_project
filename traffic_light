#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from xycar_motor.msg import xycar_motor
import cv2
import numpy as np

class CompressedTrafficLightDetector:
    def __init__(self):
        rospy.init_node('compressed_traffic_light_detector', anonymous=True)

        # 압축 이미지 구독
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback)

        # 차량 제어 퍼블리셔
        self.motor_pub = rospy.Publisher("/xycar_motor", xycar_motor, queue_size=1)

        self.green_detected = False
        self.drive_msg = xycar_motor()
        self.drive_msg.angle = 0
        self.drive_msg.speed = 0

    def image_callback(self, msg):
        # 압축 이미지 → OpenCV 이미지로 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # ROI: 신호등 위치 (해당 위치는 Unity 환경에 따라 조정 필요)
        roi = frame[0:100, 500:600]  # 예시: 상단 오른쪽 영역

        # HSV 색공간으로 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 초록색 마스크 범위
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 초록색 면적이 일정 이상이면 출발
        green_area = cv2.countNonZero(mask)
        if green_area > 200:
            self.green_detected = True

        if self.green_detected:
            self.drive_msg.angle = 0.0
            self.drive_msg.speed = 20.0
        else:
            self.drive_msg.speed = 0.0

        self.motor_pub.publish(self.drive_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CompressedTrafficLightDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
