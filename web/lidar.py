import os
import time
from rplidar import RPLidar

from config import (
    robot_data,
    LIDAR_ENABLED,
    LIDAR_PORT,
    LIDAR_BAUDRATE,
    LIDAR_MIN_DISTANCE_MM,
    LIDAR_MAX_DISTANCE_MM,
    LIDAR_DANGER_DISTANCE_MM
)

last_points_by_angle = {}


def angle_in_range(angle, start, end):
    if start <= end:
        return start <= angle <= end

    return angle >= start or angle <= end


def safe_disconnect(lidar):
    if lidar is None:
        return

    try:
        lidar.stop()
    except Exception:
        pass

    try:
        lidar.stop_motor()
    except Exception:
        pass

    try:
        lidar.disconnect()
    except Exception:
        pass


def update_lidar_data(scan):
    now_time = time.time()

    for quality, angle, distance in scan:
        if quality <= 0:
            continue

        if distance is None:
            continue

        angle = float(angle) % 360
        distance = float(distance)

        if distance < LIDAR_MIN_DISTANCE_MM:
            continue

        if distance > LIDAR_MAX_DISTANCE_MM:
            continue

        # 0.25 grade = mai multe puncte vizibile
        angle_key = round(angle * 4) / 4

        last_points_by_angle[angle_key] = {
            "angle": angle,
            "distance": distance,
            "quality": quality,
            "time": now_time
        }

    # păstrăm punctele 3 secunde ca să se vadă mobila/obiectele
    old_keys = []

    for key, point in last_points_by_angle.items():
        if now_time - point["time"] > 3.0:
            old_keys.append(key)

    for key in old_keys:
        del last_points_by_angle[key]

    clean_points = []

    for key in sorted(last_points_by_angle.keys()):
        point = last_points_by_angle[key]
        clean_points.append({
            "angle": round(point["angle"], 2),
            "distance": round(point["distance"], 1),
            "quality": int(point["quality"])
        })

    front = []
    left = []
    right = []
    back = []

    for point in clean_points:
        angle = point["angle"]
        distance = point["distance"]

        if angle_in_range(angle, 330, 30):
            front.append(distance)
        elif 60 <= angle <= 120:
            left.append(distance)
        elif 150 <= angle <= 210:
            back.append(distance)
        elif 240 <= angle <= 300:
            right.append(distance)

    min_front = min(front) if front else None
    min_left = min(left) if left else None
    min_right = min(right) if right else None
    min_back = min(back) if back else None

    robot_data["lidar"]["connected"] = True
    robot_data["lidar"]["error"] = None
    robot_data["lidar"]["name"] = "RoboPeak RPLIDAR A1M8-R5"
    robot_data["lidar"]["port"] = LIDAR_PORT
    robot_data["lidar"]["points"] = clean_points
    robot_data["lidar"]["points_count"] = len(clean_points)

    robot_data["lidar"]["min_front_mm"] = round(min_front) if min_front else None
    robot_data["lidar"]["min_left_mm"] = round(min_left) if min_left else None
    robot_data["lidar"]["min_right_mm"] = round(min_right) if min_right else None
    robot_data["lidar"]["min_back_mm"] = round(min_back) if min_back else None

    robot_data["lidar"]["obstacle_front"] = min_front is not None and min_front <= LIDAR_DANGER_DISTANCE_MM
    robot_data["lidar"]["obstacle_left"] = min_left is not None and min_left <= LIDAR_DANGER_DISTANCE_MM
    robot_data["lidar"]["obstacle_right"] = min_right is not None and min_right <= LIDAR_DANGER_DISTANCE_MM
    robot_data["lidar"]["obstacle_back"] = min_back is not None and min_back <= LIDAR_DANGER_DISTANCE_MM

    robot_data["lidar"]["last_update"] = time.strftime("%H:%M:%S")


def lidar_loop():
    if not LIDAR_ENABLED:
        robot_data["lidar"]["connected"] = False
        robot_data["lidar"]["error"] = "LIDAR dezactivat din config.py"
        return

    while True:
        lidar = None

        try:
            robot_data["lidar"]["connected"] = False
            robot_data["lidar"]["error"] = f"Conectare LIDAR la {LIDAR_PORT}..."
            robot_data["lidar"]["port"] = LIDAR_PORT

            if not os.path.exists(LIDAR_PORT):
                raise Exception(f"Portul LIDAR nu există: {LIDAR_PORT}")

            print("Conectare LIDAR la", LIDAR_PORT)

            lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=3)

            try:
                lidar.stop()
                lidar.stop_motor()
                time.sleep(0.5)
            except Exception:
                pass

            try:
                lidar.reset()
                time.sleep(2)
            except Exception:
                pass

            try:
                lidar.clear_input()
            except Exception:
                pass

            info = lidar.get_info()
            print("✅ LIDAR info:", info)

            try:
                health = lidar.get_health()
                print("✅ LIDAR health:", health)
            except Exception:
                pass

            lidar.start_motor()
            time.sleep(2)

            robot_data["lidar"]["connected"] = True
            robot_data["lidar"]["error"] = None

            print("✅ LIDAR conectat corect:", LIDAR_PORT)

            for scan in lidar.iter_scans(max_buf_meas=8000, min_len=3):
                update_lidar_data(scan)

        except Exception as error:
            robot_data["lidar"]["connected"] = False
            robot_data["lidar"]["error"] = str(error)
            print("Eroare LIDAR:", error)

            safe_disconnect(lidar)
            time.sleep(5)
