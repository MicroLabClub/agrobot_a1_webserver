import math
import threading
import time

# Porturi fixe
SERIAL_PORT = "/dev/ttyACM0"   # control motoare / MAVLink
BAUDRATE = 115200

LIDAR_PORT = "/dev/ttyUSB0"    # RoboPeak RPLIDAR A1M8-R5
LIDAR_BAUDRATE = 115200

# Control mașină
STEERING_CH = 1
THROTTLE_CH = 3

PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900

THROTTLE_PWM_STEP = 150
STEERING_PWM_STEP = 220

DEADMAN_SECONDS = 0.7

# Baterie: 2 acumulatoare 12V în serie = 24V
BATTERY_MIN_VOLTAGE = 21.0
BATTERY_MAX_VOLTAGE = 25.6
BATTERY_LOW_VOLTAGE = 22.0

# LIDAR
LIDAR_ENABLED = True
LIDAR_MIN_DISTANCE_MM = 80
LIDAR_MAX_DISTANCE_MM = 12000
LIDAR_DANGER_DISTANCE_MM = 600

runtime = {
    "master": None,
    "mav_lock": threading.Lock(),
    "last_battery_history_minute": None
}

robot_data = {
    "connected": False,
    "error": None,
    "updated": None,

    "mavlink": {
        "port": SERIAL_PORT,
        "baudrate": BAUDRATE,
        "target_system": None,
        "target_component": None
    },

    "battery": {
        "voltage_v": None,
        "remaining_percent": None,
        "capacity_mah": 15000,
        "low_voltage": BATTERY_LOW_VOLTAGE,
        "max_voltage": BATTERY_MAX_VOLTAGE,
        "min_voltage": BATTERY_MIN_VOLTAGE,
        "history": []
    },

    "gps": {
        "lat": None,
        "lon": None,
        "abs_alt_m": None,
        "rel_alt_m": None,
        "satellites": None,
        "fix_type": None
    },

    "movement": {
        "speed_m_s": None,
        "speed_km_h": None,
        "heading_deg": None,
        "roll_deg": None,
        "pitch_deg": None,
        "yaw_deg": None
    },

    "mission": {
        "waypoint_count": None,
        "current_waypoint": None,
        "wp_speed": 1.0,
        "distance_to_next_m": None
    },

    "safety": {
        "armed": None,
        "health_ok": None,
        "gps_ok": None,
        "battery_ok": None,
        "rc_available": None,
        "rc_signal_percent": None
    },

    "control": {
        "mode": "rc",
        "enabled": False,
        "command": "telecomanda",
        "rear_motor": 0,
        "front_servo": 0,
        "analog_throttle": 0.0,
        "analog_steering": 0.0,
        "throttle_pwm": PWM_MID,
        "steering_pwm": PWM_MID,
        "last_command_time": 0,
        "manual_error": None
    },

    "lidar": {
        "connected": False,
        "error": None,
        "name": "RoboPeak RPLIDAR A1M8-R5",
        "port": LIDAR_PORT,
        "points_count": 0,
        "min_front_mm": None,
        "min_left_mm": None,
        "min_right_mm": None,
        "min_back_mm": None,
        "obstacle_front": False,
        "obstacle_left": False,
        "obstacle_right": False,
        "obstacle_back": False,
        "last_update": None,
        "points": []
    },

    "rc_raw": {
        "rssi": None,
        "signal_percent": None,
        "last_update": None,
        "channels": {
            "ch1": None,
            "ch2": None,
            "ch3": None,
            "ch4": None,
            "ch5": None,
            "ch6": None,
            "ch7": None,
            "ch8": None,
            "ch9": None,
            "ch10": None,
            "ch11": None,
            "ch12": None,
            "ch13": None,
            "ch14": None,
            "ch15": None,
            "ch16": None
        }
    }
}


def now():
    return time.time()


def clean_float(value):
    if value is None:
        return None

    try:
        value = float(value)
    except Exception:
        return None

    if math.isnan(value) or math.isinf(value):
        return None

    return value


def normalize_percent(percent):
    percent = clean_float(percent)

    if percent is None:
        return None

    if percent <= 1:
        percent = percent * 100

    if percent > 100:
        percent = 100

    if percent < 0:
        percent = 0

    return round(percent)


def pwm_limit(value):
    value = int(value)

    if value < PWM_MIN:
        value = PWM_MIN

    if value > PWM_MAX:
        value = PWM_MAX

    return value
