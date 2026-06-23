import time
from pymavlink import mavutil
from config import robot_data


def handle_heartbeat(msg):
    base_mode = getattr(msg, "base_mode", 0)
    armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    robot_data["safety"]["armed"] = armed


def handle_mission_count(msg):
    count = getattr(msg, "count", None)
    robot_data["mission"]["waypoint_count"] = count


def handle_rc_channels(msg):
    channels = robot_data["rc_raw"]["channels"]

    for i in range(1, 17):
        attr = f"chan{i}_raw"
        key = f"ch{i}"
        value = getattr(msg, attr, None)

        if value is not None and value != 0:
            channels[key] = value

    rssi = getattr(msg, "rssi", None)

    robot_data["rc_raw"]["rssi"] = rssi
    robot_data["rc_raw"]["last_update"] = time.strftime("%H:%M:%S")

    if rssi is not None and rssi != 255:
        percent = int((rssi / 254.0) * 100)
        robot_data["rc_raw"]["signal_percent"] = percent
        robot_data["safety"]["rc_signal_percent"] = percent
        robot_data["safety"]["rc_available"] = percent > 5
    else:
        robot_data["rc_raw"]["signal_percent"] = None
        robot_data["safety"]["rc_signal_percent"] = None
        robot_data["safety"]["rc_available"] = None
