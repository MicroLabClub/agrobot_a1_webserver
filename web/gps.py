from config import robot_data


def handle_gps_raw_int(msg):
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


def handle_global_position_int(msg):
    rel_alt = getattr(msg, "relative_alt", None)

    if rel_alt is not None:
        robot_data["gps"]["rel_alt_m"] = rel_alt / 1000.0
