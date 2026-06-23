#!/usr/bin/env python3

import threading
import time
from flask import Flask, jsonify, render_template, request
from pymavlink import mavutil

from config import (
    SERIAL_PORT,
    BAUDRATE,
    PWM_MID,
    robot_data,
    runtime,
    now
)

from battery import handle_sys_status
from gps import handle_gps_raw_int, handle_global_position_int
from movement import handle_attitude, handle_vfr_hud
from mission import handle_heartbeat, handle_mission_count, handle_rc_channels
from control import (
    fix_target,
    control_loop,
    set_drive_command,
    set_analog_drive,
    release_rc_override,
    arm_robot,
    disarm_robot,
    set_manual_mode
)
from lidar import lidar_loop

app = Flask(__name__)


def request_message_interval(message_id, hz):
    master = runtime["master"]

    if master is None:
        return

    if hz <= 0:
        return

    fix_target()

    interval_us = int(1000000 / hz)

    try:
        with runtime["mav_lock"]:
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
    master = runtime["master"]

    if master is None:
        return

    fix_target()

    try:
        with runtime["mav_lock"]:
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


def connect_robot():
    robot_data["connected"] = False
    robot_data["error"] = f"Conectare MAVLink la {SERIAL_PORT}:{BAUDRATE}..."

    master = mavutil.mavlink_connection(
        SERIAL_PORT,
        baud=BAUDRATE,
        autoreconnect=True
    )

    master.wait_heartbeat(timeout=15)

    runtime["master"] = master

    fix_target()

    robot_data["mavlink"]["port"] = SERIAL_PORT
    robot_data["mavlink"]["baudrate"] = BAUDRATE
    robot_data["mavlink"]["target_system"] = master.target_system
    robot_data["mavlink"]["target_component"] = master.target_component

    robot_data["connected"] = True
    robot_data["error"] = None
    robot_data["updated"] = now()

    print("✅ Conectat la robot")
    print("Port:", SERIAL_PORT)
    print("Baud:", BAUDRATE)
    print("Target system:", master.target_system)
    print("Target component:", master.target_component)

    request_data_streams()

    return master


def mavlink_connect_loop():
    while True:
        try:
            master = connect_robot()
            mavlink_read_loop(master)

        except Exception as e:
            robot_data["connected"] = False
            robot_data["error"] = str(e)
            print("Eroare MAVLink:", e)
            time.sleep(3)


def mavlink_read_loop(master):
    while True:
        msg = master.recv_match(blocking=True, timeout=1)

        if msg is None:
            continue

        msg_type = msg.get_type()
        robot_data["updated"] = now()

        if msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "SYS_STATUS":
            handle_sys_status(msg)
        elif msg_type == "GPS_RAW_INT":
            handle_gps_raw_int(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            handle_global_position_int(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)
        elif msg_type == "VFR_HUD":
            handle_vfr_hud(msg)
        elif msg_type == "RC_CHANNELS":
            handle_rc_channels(msg)
        elif msg_type == "MISSION_COUNT":
            handle_mission_count(msg)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/robot")
def api_robot():
    return jsonify(robot_data)


@app.route("/api/lidar")
def api_lidar():
    return jsonify(robot_data["lidar"])


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

        ok, error = set_manual_mode()

        if error:
            robot_data["control"]["manual_error"] = error

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

    t3 = threading.Thread(target=lidar_loop, daemon=True)
    t3.start()

    app.run(host="0.0.0.0", port=5000)
