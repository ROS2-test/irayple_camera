"""Общий путь к markers_config.txt в src workspace."""
import os

from ament_index_python.packages import get_package_share_directory


def markers_config_path():
    """src/irayple/pose/markers_config.txt — один файл для SLAM и локализации."""
    ws = os.environ.get('ROS2_WS')
    if not ws:
        share = get_package_share_directory('irayple')
        candidate = share
        for _ in range(6):
            if os.path.basename(candidate) == 'install':
                ws = os.path.dirname(candidate)
                break
            parent = os.path.dirname(candidate)
            if parent == candidate:
                break
            candidate = parent
    if not ws:
        ws = os.path.expanduser('~/ros2_ws')

    src_path = os.path.join(ws, 'src', 'irayple', 'pose', 'markers_config.txt')
    if os.path.isdir(os.path.dirname(src_path)):
        return src_path

    return os.path.join(
        get_package_share_directory('irayple'), 'pose', 'markers_config.txt'
    )
