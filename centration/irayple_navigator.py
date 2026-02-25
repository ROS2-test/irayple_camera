#!/usr/bin/env python3
import rclpy
import math
import re
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class IraypleNavigator(Node):
    """Нода для приезда к 0,0,0"""

    def __init___(self):
        rclpy.initnode("irayple_navigator")
        self.max_speed = 0.3
        self.max_ang_speed = 0.5
        self.dist_tolerance = 5
        self.angle_tolerance = 3
        self.rate = 100

        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # Флаг получения данных
        self.get_data = False

        # Флаг достижения цели
        self.done = False

        self.subscribtion_irayple = self.create_subscription(
            String,
            "/irayple",
            self.irayple_callback,
            10
        )

        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)








def irayple_callback(self, msg):
    """Callback чтения /irayple."""
    self.get_data = True
    self.x = msg.x
    self.y = msg.y
    self.theta = msg.information
    self.byte = msg.byte

def timer_callback(self):
    """Callback для таймера."""
