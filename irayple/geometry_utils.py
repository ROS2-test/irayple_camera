"""Утилиты геометрии и монтажного угла камеры irayple."""
import math

# Камера повёрнута относительно продольной оси робота против часовой на 45°.
CAMERA_YAW_DEG = 135.0
CAMERA_YAW_RAD = math.radians(CAMERA_YAW_DEG)
# information камеры
CAMERA_YAW_DECI = CAMERA_YAW_DEG / 0.1


def euler_from_quaternion(quaternion):
    """Convert quaternion (x, y, z, w) to roll, pitch, yaw."""
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def camera_xy_to_robot(x_cam, y_cam, yaw_rad=CAMERA_YAW_RAD):
    """Смещение из СК камеры в СК робота (yaw против часовой)."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x_r = x_cam * c - y_cam * s
    y_r = x_cam * s + y_cam * c
    return x_r, y_r


def camera_information_to_robot(information, yaw_deci=CAMERA_YAW_DECI):
    """information камеры (1..3599, 0.1) → эквивалент в СК робота."""
    # Вычитаем монтажный угол, сохраняем диапазон 1..3599.
    return (float(information) - 1.0 - yaw_deci) % 3600.0 + 1.0


def camera_raw_to_robot(x, y, information):
    """Сырой пакет: x/y без изменений, information минус монтажный угол 45."""
    info_r = camera_information_to_robot(information)
    return int(x), int(y), int(round(info_r))
