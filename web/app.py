#!/usr/bin/env python3

import math
import threading
import time
from flask import Flask, jsonify, render_template, request
from pymavlink import mavutil

app = Flask(__name__)

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# Mașină / rover:
# CH1 = servo față / direcție
# CH3 = motor spate / accelerație înainte-înapoi
STEERING_CH = 1
THROTTLE_CH = 3

PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900

# Putere pentru motor și servo.
# Dacă nu pornește motorul, mărește THROTTLE_PWM_STEP la 160 sau 200.
THROTTLE_PWM_STEP = 150
STEERING_PWM_STEP = 220

# Dacă nu mai vine comandă de la joystick, oprește automat
DEADMAN_SECONDS = 0.7

master = None
mav_lock = threading.Lock()
last_battery_history_minute = None

robot_data = {
    "connected": False,
    "error": None,
    "updated": None,

    "battery": {
        "voltage_v": None,
        "remaining_percent": None,
        "capacity_mah": 15000,
        "low_voltage": 16.0,
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


def add_battery_history(voltage, percent):
    global last_battery_history_minute

    voltage = clean_float(voltage)
    percent = normalize_percent(percent)

    if voltage is None:
        return

    current_minute = time.strftime("%H:%M")

    if last_battery_history_minute == current_minute:
        return

    last_battery_history_minute = current_minute

    robot_data["battery"]["history"].append({
        "time": current_minute,
        "voltage_v": round(voltage, 2),
        "percent": percent
    })

    if len(robot_data["battery"]["history"]) > 300:
        robot_data["battery"]["history"] = robot_data["battery"]["history"][-300:]


def set_analog_drive(throttle, steering):
    """
    Joystick analog:
    throttle: -1.0 ... 1.0
        1.0 = înainte
       -1.0 = înapoi

    steering: -1.0 ... 1.0
       -1.0 = stânga
        1.0 = dreapta

    Exemplu:
    throttle 0.70 + steering 0.50 = înainte și dreapta
    """

    try:
        throttle = float(throttle)
    except Exception:
        throttle = 0.0

    try:
        steering = float(steering)
    except Exception:
        steering = 0.0

    if throttle > 1:
        throttle = 1
    if throttle < -1:
        throttle = -1

    if steering > 1:
        steering = 1
    if steering < -1:
        steering = -1

    if robot_data["control"]["mode"] != "web":
        throttle = 0.0
        steering = 0.0
        command = "telecomanda"
    else:
        if abs(throttle) < 0.05 and abs(steering) < 0.05:
            command = "stop"
        else:
            command = "joystick"

    throttle_pwm = PWM_MID + int(throttle * THROTTLE_PWM_STEP)
    steering_pwm = PWM_MID + int(steering * STEERING_PWM_STEP)

    robot_data["control"]["command"] = command
    robot_data["control"]["analog_throttle"] = round(throttle, 2)
    robot_data["control"]["analog_steering"] = round(steering, 2)

    robot_data["control"]["rear_motor"] = int(throttle * THROTTLE_PWM_STEP)
    robot_data["control"]["front_servo"] = int(steering * STEERING_PWM_STEP)

    robot_data["control"]["throttle_pwm"] = pwm_limit(throttle_pwm)
    robot_data["control"]["steering_pwm"] = pwm_limit(steering_pwm)

    robot_data["control"]["last_command_time"] = now()


def set_drive_command(command):
    """
    Compatibilitate cu butoane vechi, dacă mai există în browser/cache.
    """

    command = str(command).lower()

    if command == "forward":
        set_analog_drive(1.0, 0.0)

    elif command == "backward":
        set_analog_drive(-1.0, 0.0)

    elif command == "left":
        set_analog_drive(0.0, -1.0)

    elif command == "right":
        set_analog_drive(0.0, 1.0)

    else:
        set_analog_drive(0.0, 0.0)


def send_rc_override(steering_pwm, throttle_pwm):
    global master

    if master is None:
        robot_data["control"]["manual_error"] = "MAVLink nu este conectat"
        return

    channels = [0] * 8

    channels[STEERING_CH - 1] = pwm_limit(steering_pwm)
    channels[THROTTLE_CH - 1] = pwm_limit(throttle_pwm)

    try:
        with mav_lock:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7]
            )

        robot_data["control"]["manual_error"] = None

    except Exception as e:
        robot_data["control"]["manual_error"] = str(e)


def release_rc_override():
    global master

    if master is None:
        return

    try:
        with mav_lock:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
    except Exception:
        pass


def arm_robot():
    global master

    if master is None:
        return False, "MAVLink nu este conectat"

    try:
        with mav_lock:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0
            )

        return True, None

    except Exception as e:
        return False, str(e)


def disarm_robot():
    global master

    if master is None:
        return False, "MAVLink nu este conectat"

    try:
        with mav_lock:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            )

        return True, None

    except Exception as e:
        return False, str(e)


def set_manual_mode():
    global master

    if master is None:
        return False, "MAVLink nu este conectat"

    try:
        with mav_lock:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0
            )

        return True, None

    except Exception as e:
        return False, str(e)


def request_message_interval(message_id, hz):
    global master

    if master is None:
        return

    if hz <= 0:
        return

    interval_us = int(1000000 / hz)

    try:
        with mav_lock:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0,
                0,
                0,
                0,
                0
            )
    except Exception:
        pass


def request_data_streams():
    global master

    if master is None:
        return

    try:
        with mav_lock:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1
            )
    except Exception:
        pass

    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5)


def mavlink_connect_loop():
    global master

    while True:
        try:
            robot_data["connected"] = False
            robot_data["error"] = f"Conectare MAVLink la {SERIAL_PORT}:{BAUDRATE}..."

            master = mavutil.mavlink_connection(
                SERIAL_PORT,
                baud=BAUDRATE,
                autoreconnect=True
            )

            master.wait_heartbeat(timeout=15)

            robot_data["connected"] = True
            robot_data["error"] = None
            robot_data["updated"] = now()

            request_data_streams()
            mavlink_read_loop()

        except Exception as e:
            robot_data["connected"] = False
            robot_data["error"] = str(e)
            time.sleep(3)


def mavlink_read_loop():
    global master

    while True:
        msg = master.recv_match(blocking=True, timeout=1)

        if msg is None:
            continue

        msg_type = msg.get_type()
        robot_data["updated"] = now()

        if msg_type == "HEARTBEAT":
            base_mode = getattr(msg, "base_mode", 0)
            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            robot_data["safety"]["armed"] = armed

        elif msg_type == "SYS_STATUS":
            voltage_mv = getattr(msg, "voltage_battery", -1)
            battery_remaining = getattr(msg, "battery_remaining", -1)

            voltage_v = None
            percent = None

            if voltage_mv is not None and voltage_mv > 0:
                voltage_v = voltage_mv / 1000.0
                robot_data["battery"]["voltage_v"] = voltage_v

            if battery_remaining is not None and battery_remaining >= 0:
                percent = normalize_percent(battery_remaining)
                robot_data["battery"]["remaining_percent"] = percent

            add_battery_history(voltage_v, percent)

            if voltage_v is not None:
                robot_data["safety"]["battery_ok"] = voltage_v >= robot_data["battery"]["low_voltage"]

        elif msg_type == "GPS_RAW_INT":
            fix_type = getattr(msg, "fix_type", 0)
            satellites = getattr(msg, "satellites_visible", None)
            lat_raw = getattr(msg, "lat", 0)
            lon_raw = getattr(msg, "lon", 0)
            alt_raw = getattr(msg, "alt", 0)

            robot_data["gps"]["fix_type"] = str(fix_type)
            robot_data["gps"]["satellites"] = satellites
            robot_data["safety"]["gps_ok"] = fix_type >= 3

            if lat_raw != 0 and lon_raw != 0:
                robot_data["gps"]["lat"] = lat_raw / 10000000.0
                robot_data["gps"]["lon"] = lon_raw / 10000000.0

            robot_data["gps"]["abs_alt_m"] = alt_raw / 1000.0

        elif msg_type == "GLOBAL_POSITION_INT":
            rel_alt = getattr(msg, "relative_alt", None)

            if rel_alt is not None:
                robot_data["gps"]["rel_alt_m"] = rel_alt / 1000.0

        elif msg_type == "ATTITUDE":
            roll = getattr(msg, "roll", None)
            pitch = getattr(msg, "pitch", None)
            yaw = getattr(msg, "yaw", None)

            if roll is not None:
                robot_data["movement"]["roll_deg"] = math.degrees(roll)

            if pitch is not None:
                robot_data["movement"]["pitch_deg"] = math.degrees(pitch)

            if yaw is not None:
                robot_data["movement"]["yaw_deg"] = math.degrees(yaw)

        elif msg_type == "VFR_HUD":
            groundspeed = getattr(msg, "groundspeed", None)
            heading = getattr(msg, "heading", None)

            if groundspeed is not None:
                robot_data["movement"]["speed_m_s"] = groundspeed
                robot_data["movement"]["speed_km_h"] = groundspeed * 3.6

            if heading is not None:
                robot_data["movement"]["heading_deg"] = heading

        elif msg_type == "RC_CHANNELS":
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

        elif msg_type == "MISSION_COUNT":
            count = getattr(msg, "count", None)
            robot_data["mission"]["waypoint_count"] = count


def control_loop():
    while True:
        try:
            if robot_data["control"]["mode"] == "rc":
                robot_data["control"]["enabled"] = False
                robot_data["control"]["command"] = "telecomanda"
                robot_data["control"]["rear_motor"] = 0
                robot_data["control"]["front_servo"] = 0
                robot_data["control"]["analog_throttle"] = 0.0
                robot_data["control"]["analog_steering"] = 0.0
                robot_data["control"]["throttle_pwm"] = PWM_MID
                robot_data["control"]["steering_pwm"] = PWM_MID
                release_rc_override()
                time.sleep(0.2)
                continue

            robot_data["control"]["enabled"] = True

            last_time = robot_data["control"]["last_command_time"]
            elapsed = now() - last_time

            if elapsed > DEADMAN_SECONDS:
                robot_data["control"]["command"] = "stop"
                robot_data["control"]["rear_motor"] = 0
                robot_data["control"]["front_servo"] = 0
                robot_data["control"]["analog_throttle"] = 0.0
                robot_data["control"]["analog_steering"] = 0.0
                robot_data["control"]["throttle_pwm"] = PWM_MID
                robot_data["control"]["steering_pwm"] = PWM_MID

            steering = robot_data["control"]["steering_pwm"]
            throttle = robot_data["control"]["throttle_pwm"]

            send_rc_override(steering, throttle)

        except Exception as e:
            robot_data["control"]["manual_error"] = str(e)

        time.sleep(0.1)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/robot")
def api_robot():
    return jsonify(robot_data)


@app.route("/api/control", methods=["POST"])
def api_control():
    data = request.get_json(silent=True) or {}

    command = data.get("command", "stop")
    allowed = ["forward", "backward", "left", "right", "stop"]

    if command not in allowed:
        command = "stop"

    set_drive_command(command)

    return jsonify({
        "ok": True,
        "control": robot_data["control"]
    })


@app.route("/api/control_analog", methods=["POST"])
def api_control_analog():
    data = request.get_json(silent=True) or {}

    throttle = data.get("throttle", 0)
    steering = data.get("steering", 0)

    set_analog_drive(throttle, steering)

    return jsonify({
        "ok": True,
        "control": robot_data["control"]
    })


@app.route("/api/control_mode", methods=["POST"])
def api_control_mode():
    data = request.get_json(silent=True) or {}

    mode = data.get("mode", "rc")

    if mode not in ["web", "rc"]:
        mode = "rc"

    robot_data["control"]["mode"] = mode

    if mode == "rc":
        robot_data["control"]["enabled"] = False
        robot_data["control"]["command"] = "telecomanda"
        robot_data["control"]["rear_motor"] = 0
        robot_data["control"]["front_servo"] = 0
        robot_data["control"]["analog_throttle"] = 0.0
        robot_data["control"]["analog_steering"] = 0.0
        robot_data["control"]["throttle_pwm"] = PWM_MID
        robot_data["control"]["steering_pwm"] = PWM_MID
        release_rc_override()

    else:
        robot_data["control"]["enabled"] = True
        robot_data["control"]["command"] = "stop"
        robot_data["control"]["rear_motor"] = 0
        robot_data["control"]["front_servo"] = 0
        robot_data["control"]["analog_throttle"] = 0.0
        robot_data["control"]["analog_steering"] = 0.0
        robot_data["control"]["throttle_pwm"] = PWM_MID
        robot_data["control"]["steering_pwm"] = PWM_MID
        robot_data["control"]["last_command_time"] = now()

    return jsonify({
        "ok": True,
        "mode": mode,
        "control": robot_data["control"]
    })


@app.route("/api/arm", methods=["POST"])
def api_arm():
    ok, error = arm_robot()

    if error:
        robot_data["control"]["manual_error"] = error

    return jsonify({
        "ok": ok,
        "error": error
    })


@app.route("/api/disarm", methods=["POST"])
def api_disarm():
    ok, error = disarm_robot()

    if error:
        robot_data["control"]["manual_error"] = error

    return jsonify({
        "ok": ok,
        "error": error
    })


@app.route("/api/manual_mode", methods=["POST"])
def api_manual_mode():
    ok, error = set_manual_mode()

    if error:
        robot_data["control"]["manual_error"] = error

    return jsonify({
        "ok": ok,
        "error": error
    })


@app.route("/api/release_rc", methods=["POST"])
def api_release_rc():
    release_rc_override()

    return jsonify({
        "ok": True
    })


if __name__ == "__main__":
    t1 = threading.Thread(target=mavlink_connect_loop, daemon=True)
    t1.start()

    t2 = threading.Thread(target=control_loop, daemon=True)
    t2.start()

    app.run(host="0.0.0.0", port=5000)
