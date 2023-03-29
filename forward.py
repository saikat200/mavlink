import time
from pymavlink import mavutil

# Set up the connection to the autopilot
master = mavutil.mavlink_connection('com4')
master.wait_heartbeat()

# Choose a mode
#Try: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
mode = 'ALT_HOLD'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]

# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
print(mode)

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

# Set mode to manual submarine mode
master.mav.manual_control_send(
    master.target_system,
    500,
    -500,
    250,
    500,
    0)

# Set thrust values for forward motion
forward_thrust = 400 # replace with your desired thrust value
yaw_thrust = 400 # replace with your desired thrust value

# Set duration of forward motion in seconds
duration = 2

# Send thrust values for duration of motion
start_time = time.time()
while time.time() - start_time < duration:
    master.mav.manual_control_send(
        master.target_system,
        forward_thrust,
        0,
        yaw_thrust,
        0,
        0
    )
    print('Moving forward...')

# Stop the vehicle
master.mav.manual_control_send(
    master.target_system,
    1500,
    0,
    1500,
    0,
    0
)
print('Stopped.')
