#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, rospy, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

start_point = None
end_point = None
roi_selected = False

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def mouse_callback(event, x, y, flags, param):
    global start_point, end_point, roi_selected

    if event == cv2.EVENT_LBUTTONDOWN:
        if start_point is None:
            start_point = (x, y)
            rospy.loginfo(f"Start point: {start_point}")
        elif end_point is None:
            end_point = (x, y)
            roi_selected = True
            rospy.loginfo(f"End point: {end_point}")

rospy.init_node('roi_box_selector', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

rospy.wait_for_message("/usb_cam/image_raw/", Image)
print("Camera Ready - Click two points to select ROI rectangle")

cv2.namedWindow("camera")
cv2.setMouseCallback("camera", mouse_callback)

while not rospy.is_shutdown():
    if cv_image.size != 0:
        display = cv_image.copy()

        # ROI 시각화
        if start_point and not end_point:
            cv2.circle(display, start_point, 5, (0, 255, 255), -1)

        if start_point and end_point:
            cv2.rectangle(display, start_point, end_point, (0, 255, 0), 2)
            x1, y1 = start_point
            x2, y2 = end_point
            x_min, x_max = sorted([x1, x2])
            y_min, y_max = sorted([y1, y2])
            rospy.loginfo_throttle(2, f"ROI = frame[{y_min}:{y_max}, {x_min}:{x_max}]")

        cv2.imshow("camera", display)
        key = cv2.waitKey(1)
        if key == ord('r'):  # r 키로 초기화
            start_point = None
            end_point = None
            roi_selected = False
            rospy.loginfo("ROI reset")

# 찾은 초록불 위치

[INFO] [1747428079.732963]: ROI = frame[63:97, 392:431]
