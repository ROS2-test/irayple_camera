#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # парсер данных с нижней камеры
        Node(
            package='irayple',
            executable='irayple_parser',
            name='irayple_parser',
            parameters=[
                {'host': '192.168.0.14'},
                {'port': 3000},
                {'localip': '192.168.0.200'},
                {'topic': '/irayplelower'},
            ]
        ),
        # верхняя камера
        Node(
            package='irayple',
            executable='irayple_parser',
            name='irayple_parser_upper',
            parameters=[
                {'host': '192.168.0.15'},
                {'port': 3000},
                {'localip': '192.168.0.200'},
                {'topic': '/iraypleupper'},
            ]
        ),
        # публикация позы по qr для нижней камеры
        Node(
            package='irayple',
            executable='irayple_pose_pub',
            name='irayple_pose_pub',
            parameters=[
                {'irayple_topic': '/irayplelower'},
                {'publish_initialpose': True},
            ],
        ),
        # центровка нижней камерой
        Node(
            package='irayple',
            executable='irayple_navigator2',
            name='irayple_navigator2',
            parameters=[
                {'parse_topic': '/irayplelower'},
                {'action_name': 'irayple_center_action'},
            ],
        ),
        # центровка верхней камерой
        Node(
            package='irayple',
            executable='irayple_navigator2',
            name='irayple_navigator2_upper',
            parameters=[
                {'parse_topic': '/iraypleupper'},
                {'action_name': 'irayple_center_action_upper'},
            ],
        ),
    ])
