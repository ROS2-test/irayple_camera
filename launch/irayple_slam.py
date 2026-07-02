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
            parameters=[
                {'host': '192.168.0.14'},
                {'port': 3000},
                {'localip': '192.168.0.16'},
                {'topic': '/irayplelower'},
            ],
        ),
        # публикация позы по qr
        Node(
            package='irayple',
            executable='irayple_pose_pub',
            name='irayple_pose_pub',
            parameters=[
                {'irayple_topic': '/irayplelower'},
                {'publish_initialpose': True},
            ],
        ),
        # slam
        Node(
            package='irayple',
            executable='irayple_slam',
            name='irayple_slam',
            parameters=[
                {'irayple_topic': '/irayplelower'},
            ],
        ),
    ])
