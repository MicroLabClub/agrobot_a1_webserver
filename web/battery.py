import time

from config import (
    robot_data,
    runtime,
    clean_float,
    BATTERY_MIN_VOLTAGE,
    BATTERY_MAX_VOLTAGE,
    BATTERY_LOW_VOLTAGE
)


def battery_percent_from_voltage(voltage):
    voltage = clean_float(voltage)

    if voltage is None:
        return None

    percent = (
        (voltage - BATTERY_MIN_VOLTAGE)
        / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)
    ) * 100

    if percent > 100:
        percent = 100

    if percent < 0:
        percent = 0

    return round(percent)


def add_battery_history(voltage, percent):
    voltage = clean_float(voltage)

    if voltage is None:
        return

    current_minute = time.strftime("%H:%M")

    if runtime["last_battery_history_minute"] == current_minute:
        return

    runtime["last_battery_history_minute"] = current_minute

    robot_data["battery"]["history"].append({
        "time": current_minute,
        "voltage_v": round(voltage, 2),
        "percent": percent
    })

    if len(robot_data["battery"]["history"]) > 300:
        robot_data["battery"]["history"] = robot_data["battery"]["history"][-300:]


def handle_sys_status(msg):
    voltage_mv = getattr(msg, "voltage_battery", -1)

    voltage_v = None
    percent = None

    if voltage_mv is not None and voltage_mv > 0:
        voltage_v = voltage_mv / 1000.0
        percent = battery_percent_from_voltage(voltage_v)

        robot_data["battery"]["voltage_v"] = round(voltage_v, 2)
        robot_data["battery"]["remaining_percent"] = percent

        robot_data["battery"]["min_voltage"] = BATTERY_MIN_VOLTAGE
        robot_data["battery"]["max_voltage"] = BATTERY_MAX_VOLTAGE
        robot_data["battery"]["low_voltage"] = BATTERY_LOW_VOLTAGE

        robot_data["safety"]["battery_ok"] = voltage_v >= BATTERY_LOW_VOLTAGE

        add_battery_history(voltage_v, percent)
