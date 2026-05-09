#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()

        # 订阅由模块 1 提供的高斯滤波图像
        self.image_sub = rospy.Subscriber('/image_blur', Image, self.image_callback)

        # 发布目标信息：采用 Float32MultiArray，格式为 [x, y, w, h] (外接矩形中心左上角与宽高)
        self.obj_pub = rospy.Publisher('/detected_object', Float32MultiArray, queue_size=1)
        
        # 可视化识别结果发布
        self.viz_pub = rospy.Publisher('/image_detected', Image, queue_size=1)

        # 设置HSV滤色参数 (默认提取偏红色的物体，实验时可根据瓶子颜色调整)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        # 几何与面积过滤参数
        self.min_area = rospy.get_param('~min_area', 500)

        rospy.loginfo("模块 2: Object Detection Node 已启动")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 1. 色彩空间转换 (BGR -> HSV)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. 颜色阈值分割：获取二值化 Mask (这里处理红色因跨H角两端)
        mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 3. 形态学滤波：先开运算(去噪点)后闭运算(填内部孔洞)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 4. 轮廓提取
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_bbox = None
        max_area = 0

        # 5. 寻找最大轮廓并进行几何面积过滤
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area and area > max_area:
                max_area = area
                # 获取边界框 (x, y, w, h)
                x, y, w, h = cv2.boundingRect(contour)
                best_bbox = [x, y, w, h]

        # 如果找到了符合条件的目标物
        if best_bbox is not None:
            x, y, w, h = best_bbox

            # 发布目标边界位置
            msg = Float32MultiArray()
            msg.data = [float(x), float(y), float(w), float(h)]
            self.obj_pub.publish(msg)

            # 在图像上绘制包围盒进行可视化
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, "Target", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # 未发现满足提条件目标的情况，可选用特殊标志位，比如空列表代表未检到
            msg = Float32MultiArray()
            msg.data = []
            self.obj_pub.publish(msg)

        # 发布展示
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            viz_msg.header = data.header
            self.viz_pub.publish(viz_msg)
            
            # 本地屏幕显示检测结果
            cv2.imshow("Object Detection", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
