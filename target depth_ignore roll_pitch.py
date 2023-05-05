#!/usr/bin/env python

# Script to set target depth in Ardusub 3.6 beta
# requires this commit: https://github.com/williangalvani/ardupilot/commit/85a41d5771ba9d64e594dcc0856b6a3906767c3f

from pymavlink import mavutil

# There is very likely a builtin in pymavlink for this, but I didn't find it
ALT_HOLD_MODE = 2

# This is also probably somewhere in pymavlink
def is_armed():
    try:
        return bool(master.wait_heartbeat().base_mode & 0b10000000)
    except:
        return False

def mode_is(mode):
    try:
        return bool(master.wait_heartbeat().custom_mode == mode)
    except:
        return False

def set_target_depth(depth):
    current_yaw = master.recv_match(type='ATTITUDE', blocking=True).yaw
    msg = master.mav.set_position_target_global_int_send(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, #frame
        0b110111111110, # ignore roll and pitch control
        0, 0, depth, # x, y, z position
        0, 0, 0, # x, y, z velocity (not used)
        0, 0, 0, # x, y, z acceleration (not used)
        current_yaw, 0) # yaw, yawrate



# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
print(master.wait_heartbeat())


while not is_armed():
    master.arducopter_arm()

while not mode_is(ALT_HOLD_MODE):
    master.set_mode('ALT_HOLD')
    print('hold')

set_target_depth(-0.7)