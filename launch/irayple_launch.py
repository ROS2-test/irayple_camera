#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # парсер данных с камеры
        Node(
            package='irayple',
            executable='irayple_parser',
            name='irayple_parser',
        ),
        # публикация позы по qr
        Node(
            package='irayple',
            executable='irayple_pose_pub',
            name='irayple_pose_pub',
        ),
    ])
