import time
from pymavlink import mavutil

from config import (
    robot_data,
    runtime,
    STEERING_CH,
    THROTTLE_CH,
    PWM_MID,
    THROTTLE_PWM_STEP,
    STEERING_PWM_STEP,
    DEADMAN_SECONDS,
    pwm_limit,
    now
)


def fix_target():
    master = runtime["master"]

    if master is None:
        return

    if master.target_system == 0:
        master.target_system = 1

    if master.target_component == 0:
        master.target_component = 1

    robot_data["mavlink"]["target_system"] = master.target_system
    robot_data["mavlink"]["target_component"] = master.target_component


def set_analog_drive(throttle, steering):
    try:
        throttle = float(throttle)
    except Exception:
        throttle = 0.0

    try:
        steering = float(steering)
    except Exception:
        steering = 0.0

    throttle = max(-1.0, min(1.0, throttle))
    steering = max(-1.0, min(1.0, steering))

    if robot_data["control"]["mode"] != "web":
        throttle = 0.0
        steering = 0.0
        command = "telecomanda"
    else:
        if abs(throttle) < 0.05 and abs(steering) < 0.05:
            throttle = 0.0
            steering = 0.0
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
    master = runtime["master"]

    if master is None:
        robot_data["control"]["manual_error"] = "MAVLink nu este conectat"
        return

    fix_target()

    channels = [0] * 8
    channels[STEERING_CH - 1] = pwm_limit(steering_pwm)
    channels[THROTTLE_CH - 1] = pwm_limit(throttle_pwm)

    try:
        with runtime["mav_lock"]:
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
    master = runtime["master"]

    if master is None:
        return

    fix_target()

    try:
        with runtime["mav_lock"]:
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
    except Exception:
        pass


def arm_robot():
    master = runtime["master"]

    if master is None:
        return False, "MAVLink nu este conectat"

    fix_target()

    try:
        with runtime["mav_lock"]:
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
    master = runtime["master"]

    if master is None:
        return False, "MAVLink nu este conectat"

    fix_target()

    try:
        with runtime["mav_lock"]:
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
    master = runtime["master"]

    if master is None:
        return False, "MAVLink nu este conectat"

    fix_target()

    try:
        with runtime["mav_lock"]:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0
            )

        return True, None

    except Exception as e:
        return False, str(e)


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

            elapsed = now() - robot_data["control"]["last_command_time"]

            if elapsed > DEADMAN_SECONDS:
                robot_data["control"]["command"] = "stop"
                robot_data["control"]["rear_motor"] = 0
                robot_data["control"]["front_servo"] = 0
                robot_data["control"]["analog_throttle"] = 0.0
                robot_data["control"]["analog_steering"] = 0.0
                robot_data["control"]["throttle_pwm"] = PWM_MID
                robot_data["control"]["steering_pwm"] = PWM_MID

            send_rc_override(
                robot_data["control"]["steering_pwm"],
                robot_data["control"]["throttle_pwm"]
            )

        except Exception as e:
            robot_data["control"]["manual_error"] = str(e)

        time.sleep(0.1)
