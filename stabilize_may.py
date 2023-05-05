import os
os.environ['mavlink20']=''
from pymavlink import mavutil

# Set up the connection to the vehicle
master = mavutil.mavlink_connection('com4')
master.wait_heartbeat()

# Set mode to stabilize
mode = 'STABILIZE'
master.set_mode(mode)

# Set throttle values for the motors
params = [0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 8]

# Send command to set throttle values
# Send command to set throttle values
cmd = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    cmd,
    0,
    7,  # number of parameters
    0,  # mission mode (not used)
    0,  # ROI type (not used)
    0,  # target system (not used)
    0,  # target component (not used)
    *params,  # throttle values for first four and last four motors
    0,  # ROI index (not used)
    0,  # ROI flags (not used)
    0,  # ROI coordinate frame (not used)
)
