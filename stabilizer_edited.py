import time
from pymavlink import mavutil
import keyboard

# Connect to the vehicle
master = mavutil.mavlink_connection('com4')

# Wait for the heartbeat message to ensure a connection has been established
master.wait_heartbeat()

# Set the mode to stabilize
master.set_mode('STABILIZE')

# Disable roll and pitch control
MAV_CMD_OVERRIDE_CTRL = 252
roll = 0
pitch = 0
yaw = 0
throttle = 0
mode = 1  # 0 = disable, 1 = enable
flags = 0  # unused
target = 0  # unused
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    MAV_CMD_OVERRIDE_CTRL,
    0,
    roll, pitch, yaw, throttle,
    mode, flags, target)

fix_yaw = False
current_yaw = 0
target_yaw = 0

# Continuously print the current yaw
while True:
    # Request the AHRS2 message from the autopilot
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        5, 1)

    # Wait for the AHRS2 message
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg and not keyboard.is_pressed('f'):

    # Extract the yaw from the message

        current_yaw = msg.yaw
        print('Current yaw:', current_yaw)
    if keyboard.is_pressed('f'):
        while True:
            if not fix_yaw:
                target_yaw = current_yaw
                fix_yaw = True
                print('Target yaw fixed:', target_yaw)
            # Adjust yaw value to maintain the target yaw
            if fix_yaw:
                yaw_error = target_yaw - current_yaw
                yaw = max(-45, min(yaw_error, 45))
                print('Target yaw:', target_yaw, 'Yaw error:', yaw_error, 'Yaw:', yaw)
                # Sleep for a short interval to avoid hogging CPU resources
time.sleep(0.1)







    # while True:
    #     # Check for user input to fix the target yaw
    #     if not fix_yaw and input() == 'f':
    #         target_yaw = current_yaw
    #         fix_yaw = True
    #         print('Target yaw fixed:', target_yaw)
    #
    #     # Adjust yaw value to maintain the target yaw
    #     if fix_yaw:
    #         yaw_error = target_yaw - current_yaw
    #         yaw = max(-45, min(yaw_error, 45))
    #         print('Target yaw:', target_yaw, 'Yaw error:', yaw_error, 'Yaw:', yaw)
    #
    #     # Sleep for a short interval to avoid hogging CPU resources
    #     time.sleep(0.1)

