#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImagePreprocessNode:
    def __init__(self):
        rospy.init_node('image_preprocess_node', anonymous=True)
        self.bridge = CvBridge()

        # 订阅原始图像
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # 发布处理后的图像
        self.gray_pub = rospy.Publisher('/image_gray', Image, queue_size=1)
        self.blur_pub = rospy.Publisher('/image_blur', Image, queue_size=1)

        rospy.loginfo("模块 1: Image Preprocess Node 已启动")

    def image_callback(self, data):
        try:
            # ROS Image 转换为 OpenCV 的 Mat (BGR 格式)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 1. 灰度化处理
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 2. 高斯平滑 (使用 5x5 卷积核) 过滤仿真特定噪点
        blur_image = cv2.GaussianBlur(cv_image, (5, 5), 0)

        try:
            # 将处理后的 OpenCV 图像转换回 ROS 的 sensor_msgs/Image
            gray_msg = self.bridge.cv2_to_imgmsg(gray_image, "mono8")
            blur_msg = self.bridge.cv2_to_imgmsg(blur_image, "bgr8")

            # 保持时间戳与原始数据一致
            gray_msg.header = data.header
            blur_msg.header = data.header

            self.gray_pub.publish(gray_msg)
            self.blur_pub.publish(blur_msg)
            
            # 使用 OpenCV 本地窗口直接显示图像
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Gray Image", gray_image)
            cv2.imshow("Blur Image", blur_image)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = ImagePreprocessNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
