#!/usr/bin/env python3
from pymavlink import mavutil
import time

# Connect to the vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("✅ Heartbeat received. System is alive.")

# Set GUIDED mode before arming
master.set_mode("GUIDED")
time.sleep(1)

# Arm the drone
master.arducopter_arm()
master.motors_armed_wait()
print("✅ Drone armed")

# Takeoff to 10 meters
altitude = 10
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    altitude
)
print("🚁 Taking off...")
time.sleep(10)

# Waypoint sending helper
def send_waypoint(x, y, z):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        int(0b110111111000),
        x, y, -z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

# Define scan box corners
x_start = 0       # NED X (North), corresponds to -17 in Gazebo
x_end = 90        # corresponds to 73 in Gazebo
y_start = 0       # NED Y (East), corresponds to -46
y_end = 39        # corresponds to -7
x_step = 10
y_step = 5

print("🔍 Starting snake pattern scan...")
forward = True
for y in range(y_start, y_end + 1, y_step):
    if forward:
        for x in range(x_start, x_end + 1, x_step):
            print(f"→ Going to ({x}, {y})")
            send_waypoint(x, y, altitude)
            time.sleep(6)
    else:
        for x in range(x_end, x_start - 1, -x_step):
            print(f"← Going to ({x}, {y})")
            send_waypoint(x, y, altitude)
            time.sleep(6)
    forward = not forward

# Return to initial position
print("↩ Returning to start")
send_waypoint(0, 0, altitude)
time.sleep(10)

# Land
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0,
    0, 0,
    0
)
print("🛬 Landing initiated.")
