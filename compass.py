import time
from pymavlink import mavutil

# Set up a MAVLink connection to the vehicle
connection_string = 'com4'  # Replace with your connection string
master = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message to ensure a connection is established
master.wait_heartbeat()

# Enable the compass
master.mav.command_long_send(
    master.target_system,  # Target system ID
    mavutil.mavlink.MAV_COMP_ID_ALL,  # Target component ID
    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # Command ID
    0,  # Confirmation
    1,  # Compass calibration enabled
    0, 0, 0, 0, 0, 0  # Unused parameters
)

# Wait for the compass to become calibrated
while True:
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    if msg:
        compass_calibrated = (msg.onboard_control_sensors_enabled & 2**10) != 0  # 2**10 is the integer value of the HMC5883L sensor type
        if compass_calibrated:
            break

# Continuously read the heading value
while True:
    msg = master.recv_match(type='VFR_HUD', blocking=True)
    if msg:
        heading = msg.heading
        print('Heading:', heading)
