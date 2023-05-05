import time
from pymavlink import mavutil

# Connect to the vehicle using a MAVLink connection
master = mavutil.mavlink_connection('com4')

master.wait_heartbeat()

# Wait a heartbeat before sending commands

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
# master.arducopter_arm() or:

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Define a new custom mode with fixed heading and no roll and pitch stabilization
custom_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 1 << 24 | 1 << 10

# Send the custom mode to the vehicle
msg = master.mav.command_long_send(
    0, 0,  # Target system, target component
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command ID
    0,  # Confirmation
    custom_mode,  # Custom mode
    0, 0, 0, 0, 0, 0  # Parameters
)

# Wait for the vehicle to switch to the new mode
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.custom_mode == custom_mode:
        print('mode')
        break
    time.sleep(0.1)
