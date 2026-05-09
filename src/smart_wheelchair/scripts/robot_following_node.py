#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class RobotFollowingNode:
    def __init__(self):
        rospy.init_node('robot_following_node', anonymous=True)
        self.bridge = CvBridge()

        # 状态机标识
        self.state = "DETECTING"  # DETECTING (寻找目标) 或 TRACKING (追踪目标)
        
        # OpenCV Tracker 对象初始化 (模块3要求的跟踪特征比对算法)
        self.tracker = None
        self.tracker_type = rospy.get_param('~tracker_type', 'CSRT') # 可选 CSRT 或 KCF
        
        # 图像参数
        self.img_width = 640
        self.img_height = 480
        self.center_x = self.img_width / 2.0

        # PID参数（这里使用比例控制 P）
        self.kp_angular = rospy.get_param('~kp_angular', 0.005)
        self.kp_linear = rospy.get_param('~kp_linear', 0.00005)

        # 跟踪目标的阈值
        self.target_area = rospy.get_param('~target_area', 25000) # 目标占据面积阈值（停止跟随距离）
        self.max_linear_vel = 0.5   # 限制最大跟车速度

        # 订阅由模块 1 提供的高斯滤波图像，供 Tracker 使用
        self.image_sub = rospy.Subscriber('/image_blur', Image, self.image_callback)
        # 订阅由模块 2 提供的新识别框
        self.bbox_sub = rospy.Subscriber('/detected_object', Float32MultiArray, self.bbox_callback)

        # 运动控制发布
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 状态可视化发布
        self.viz_pub = rospy.Publisher('/image_tracking', Image, queue_size=1)

        rospy.loginfo("模块 3: Robot Tracking & Following Node 已启动")

    def init_tracker(self):
        if self.tracker_type == 'CSRT':
            return cv2.TrackerCSRT_create()
        else:
            return cv2.TrackerKCF_create()

    def bbox_callback(self, msg):
        # 只有存在有效识别且状态为正在检测时才初始化跟踪
        if len(msg.data) == 4 and self.state == "DETECTING":
            self.init_bbox = (int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[3]))
            rospy.loginfo("收到初始目标框: {}".format(self.init_bbox))
            self.tracker = self.init_tracker()
            self.state = "WAIT_INIT"

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        twist = Twist() # 默认不发车

        if self.state == "WAIT_INIT":
            # 等待图像帧来初始化 Tracker
            if self.tracker.init(cv_image, self.init_bbox):
                rospy.loginfo("Tracker 初始化成功，进入 TRACKING 模式")
                self.state = "TRACKING"
            else:
                self.state = "DETECTING"

        elif self.state == "TRACKING":
            # 1. Tracker 更新
            success, bbox = self.tracker.update(cv_image)

            if success:
                # 跟踪成功，计算当前框的物理与图像参数
                x, y, w, h = [int(v) for v in bbox]
                obj_cx = x + w / 2.0
                obj_area = w * h

                # --- 2. 视觉伺服 (Visual Servoing) 运动推演 ---
                # 角速度控制：计算中轴线偏差 err_X
                err_x = self.center_x - obj_cx  
                # 如果目标在中线偏左 (Cx较小)，err_X是正数，赋予正角速度(底盘左转响应)
                twist.angular.z = self.kp_angular * err_x

                # 线速度控制：通过面积估算前后距离
                area_err = self.target_area - obj_area
                if obj_area < self.target_area:
                    # 距离太远（面积太小），向前跟进
                    twist.linear.x = min(self.kp_linear * area_err, self.max_linear_vel)
                else:
                    # 到达安全停止阈值，或已经过近
                    twist.linear.x = 0.0

                # 绘制状态标注
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(cv_image, "TRACKING: " + self.tracker_type, (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(cv_image, "err_x: {:.1f} area: {}".format(err_x, obj_area), 
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                # 跟踪失败，立即进入重新检测状态
                rospy.logwarn("目标跟丢，退回 DETECTING 模式寻找目标")
                self.state = "DETECTING"
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        
        else:
            # DETECTING：无锁定的目标原地等待
            cv2.putText(cv_image, "DETECTING...", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 发布底盘控制指令
        self.cmd_pub.publish(twist)

        # 发布可视化追踪状态图
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            viz_msg.header = data.header
            self.viz_pub.publish(viz_msg)
            
            # 本地屏幕显示追踪状态
            cv2.imshow("Tracking & Following", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = RobotFollowingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
