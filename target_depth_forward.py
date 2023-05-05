#!/usr/bin/env python

# Script to set target depth and yaw in Ardusub 3.6 beta
# requires this commit: https://github.com/williangalvani/ardupilot/commit/85a41d5771ba9d64e594dcc0856b6a3906767c3f

from pymavlink import mavutil

# There is very likely a builtin in pymavlink for this, but I didn't find it
ALT_HOLD_MODE = 2

# PWM values for the thrusters
FORWARD_PWM = 1600

# PWM channel numbers for the thrusters
FORWARD_CHANNEL = 3

# Target depth (in meters)
TARGET_DEPTH = -0.1

# Target yaw (in degrees)
TARGET_YAW = 0.0

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
    msg = master.mav.set_position_target_global_int_send(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, #frame
        0b0000111111111000,
        0,0, depth,
        0 , 0 , 0 , # x, y, z velocity in m/s (not used)
        0 , 0 , 0 , # x, y, z acceleration (not supported yet, ignored in GCS Mavlink)
        0 , 0 ) # yaw, yawrate (not supported yet, ignored in GCS Mavlink)

# Create the connection
master = mavutil.mavlink_connection('com4')

# Wait for a heartbeat from the vehicle
print(master.wait_heartbeat())

# Arm the vehicle
while not is_armed():
    master.arducopter_arm()

# Set the vehicle to Alt Hold mode
while not mode_is(ALT_HOLD_MODE):
    master.set_mode('ALT_HOLD')
    print('hold')

# Set the target depth and yaw
set_target_depth(TARGET_DEPTH)
master.mav.set_yaw_rate_send(0)
master.mav.command_long_send(
    master.target_system,  # target system
    master.target_component,  # target component
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
    0,  # confirmation
    TARGET_YAW,  # param 1 (yaw angle)
    20,  # param 2 (yaw speed)
    1,  # param 3 (direction: 1=CCW, -1=CW)
    0,  # param 4 (relative offset)
    0,  # param 5
    0,  # param 6
    0)  # param 7

# Continuously send the PWM value to the forward thruster to maintain a fixed speed
while True:
    print('forward')
    master.mav.rc_channels_override_send(
        master.target_system,  # system ID
        master.target_component,  # component ID
        FORWARD_CHANNEL,  # channel number
        FORWARD_PWM  # PWM value
    )
    set_target_depth(TARGET_DEPTH)
    # Sleep for a short period to avoid
