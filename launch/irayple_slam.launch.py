#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irayple',
            executable='irayple_parser',
            name='irayple_parser',
            output='screen',
            parameters=[
                {'host': '192.168.0.14'},
                {'port': 3000},
                {'localip': '192.168.0.200'},
                {'topic': '/irayplelower'},
            ],
        ),
        Node(
            package='irayple',
            executable='irayple_slam',
            name='irayple_slam',
            output='screen',
            parameters=[
                {'irayple_topic': '/irayplelower'},
            ],
        ),
    ])
