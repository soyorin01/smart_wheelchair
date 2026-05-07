#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能轮椅键盘遥控节点

按键说明:
    w / ↑   : 前进
    s / ↓   : 后退
    a / ←   : 左转
    d / →   : 右转
    空格    : 停止
    q       : 退出
"""

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

# 线速度与角速度配置
LINEAR_SPEED = 0.5      # m/s
ANGULAR_SPEED = 1.0     # rad/s
LINEAR_STEP = 0.1
ANGULAR_STEP = 0.1

# 键位映射
MOVE_BINDINGS = {
    'w': (1, 0),
    'W': (1, 0),
    's': (-1, 0),
    'S': (-1, 0),
    'a': (0, 1),
    'A': (0, 1),
    'd': (0, -1),
    'D': (0, -1),
}

# 方向键映射 (ESC [ A/B/C/D)
ARROW_BINDINGS = {
    'A': (1, 0),   # 上
    'B': (-1, 0),  # 下
    'C': (0, -1),  # 右
    'D': (0, 1),   # 左
}


class TeleopKeyboard:
    def __init__(self):
        rospy.init_node('teleop_keyboard', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(20)

        self.linear_speed = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """非阻塞读取单个按键（含方向键）"""
        tty.setcbreak(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        # 处理方向键 ESC 序列
        if key == '\x1b':
            key2 = sys.stdin.read(1)
            if key2 == '[':
                key3 = sys.stdin.read(1)
                return key3  # A/B/C/D
            else:
                return key2
        return key

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_status(self):
        sys.stdout.write(
            "\r当前速度: 线速度 %.2f m/s | 角速度 %.2f rad/s | 目标: %.2f m/s  %.2f rad/s "
            % (self.linear_speed, self.angular_speed, self.target_linear, self.target_angular)
        )
        sys.stdout.flush()

    def run(self):
        print("\n========== 智能轮椅键盘遥控 ==========")
        print("  w/↑ : 前进    s/↓ : 后退")
        print("  a/← : 左转    d/→ : 右转")
        print("  空格 : 停止")
        print("  u/U : 增加线速度   m/M : 减少线速度")
        print("  i/I : 增加角速度   n/N : 减少角速度")
        print("  q   : 退出")
        print("======================================\n")

        twist = Twist()
        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                if key in MOVE_BINDINGS:
                    self.target_linear = MOVE_BINDINGS[key][0] * self.linear_speed
                    self.target_angular = MOVE_BINDINGS[key][1] * self.angular_speed
                elif key in ARROW_BINDINGS:
                    self.target_linear = ARROW_BINDINGS[key][0] * self.linear_speed
                    self.target_angular = ARROW_BINDINGS[key][1] * self.angular_speed
                elif key == ' ':
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                elif key in ('u', 'U'):
                    self.linear_speed += LINEAR_STEP
                elif key in ('m', 'M'):
                    self.linear_speed = max(0.0, self.linear_speed - LINEAR_STEP)
                elif key in ('i', 'I'):
                    self.angular_speed += ANGULAR_STEP
                elif key in ('n', 'N'):
                    self.angular_speed = max(0.0, self.angular_speed - ANGULAR_STEP)
                elif key in ('q', 'Q', '\x03'):  # q 或 Ctrl+C
                    break

                twist.linear.x = self.target_linear
                twist.angular.z = self.target_angular
                self.pub.publish(twist)
                self.print_status()
                self.rate.sleep()
        finally:
            # 发送停止指令并恢复终端
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.restore_terminal()
            print("\n\n键盘遥控已退出。")


if __name__ == '__main__':
    teleop = TeleopKeyboard()
    teleop.run()
