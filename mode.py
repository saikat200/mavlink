"""
Example of how to change flight modes using pymavlink
"""

import sys
# Import mavutil
import os
os.environ['mavlink20']=''
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('com4')

# master2 = mavutil.mavlink_connection('com4')


# Wait a heartbeat before sending commands
master.wait_heartbeat()


# Set mode to stabilized
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
# Set mode to stabilized

# Set mode to stabilized
# master2.mav.command_long_send(
#     master2.target_system,
#     master2.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)
#
# # wait until arming confirmed (can manually check with master.motors_armed())
# print("Waiting for the vehicle to arm")
# master2.motors_armed_wait()
# print('Armed!')
# # Set mode to stabilized

# Choose a mode
mode = 'STABILIZE'

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

# def set_rc_channel_pwm(channel_id, pwm=1500):
#     """ Set RC channel pwm value
#     Args:
#         channel_id (TYPE): Channel ID
#         pwm (int, optional): Channel pwm value 1100-1900
#     """
#     if channel_id < 1 or channel_id > 18:
#         print("Channel does not exist.")
#         return
#
#         # Mavlink 2 supports up to 18 channels:
#         # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
#     rc_channel_values = [65535 for _ in range(18)]
#     rc_channel_values[channel_id - 1] = pwm
#     master2.mav.rc_channels_override_send(
#         master2.target_system,  # target_system
#         master2.target_component,  # target_component
#         *rc_channel_values[:8],  # channel 1-8 values
#         0, 0, 0, 0, 0, 0, 0, 0)

while True:
    # set_rc_channel_pwm(3, 1550)
    print('de')
time.sleep(0.1)
