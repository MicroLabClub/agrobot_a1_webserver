import math
from config import robot_data


def handle_attitude(msg):
    roll = getattr(msg, "roll", None)
    pitch = getattr(msg, "pitch", None)
    yaw = getattr(msg, "yaw", None)

    if roll is not None:
        robot_data["movement"]["roll_deg"] = math.degrees(roll)

    if pitch is not None:
        robot_data["movement"]["pitch_deg"] = math.degrees(pitch)

    if yaw is not None:
        robot_data["movement"]["yaw_deg"] = math.degrees(yaw)


def handle_vfr_hud(msg):
    groundspeed = getattr(msg, "groundspeed", None)
    heading = getattr(msg, "heading", None)

    if groundspeed is not None:
        robot_data["movement"]["speed_m_s"] = groundspeed
        robot_data["movement"]["speed_km_h"] = groundspeed * 3.6

    if heading is not None:
        robot_data["movement"]["heading_deg"] = heading
