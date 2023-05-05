#!/usr/bin/env python

# Script to set target yaw in Ardusub 3.6 beta
# requires this commit: https://github.com/williangalvani/ardupilot/commit/85a41d5771ba9d64e594dcc0856b6a3906767c3f

from pymavlink import mavutil
import time
STABILIZE_MODE = 0

# PWM values for the thrusters
FORWARD_PWM = 1600

# PWM channel numbers for the thrusters
FORWARD_CHANNEL = 3

# Target yaw (in degrees)
TARGET_YAW = 2.0

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

def set_target_yaw(yaw):
    depth = master.messages['SCALED_PRESSURE2'].press_diff
    msg = master.mav.set_position_target_global_int_send(
        0,
        0,  # latitude
        0,  # longitude
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # frame
        0b0000100001110000,  # type_mask: ignore roll and pitch, control yaw
        0,  # vx
        0,  # vy
        depth,  # vz: maintain current depth
        0,  # afx
        0,  # afy
        0,  # afz
        0,  # yaw_rate
        yaw  # yaw
    )

# Create the connection
master = mavutil.mavlink_connection('com4')

# Wait for a heartbeat from the vehicle
print(master.wait_heartbeat())

# Arm the vehicle
while not is_armed():
    master.arducopter_arm()

# Set the vehicle to Stabilize mode
while not mode_is(STABILIZE_MODE):
    master.set_mode('STABILIZE')
    print('stabilize')
# Set the target yaw
set_target_yaw(TARGET_YAW)
print('stabilize')


# Continuously send the PWM value to the forward thruster to maintain a fixed speed
# while True:
#     print('forward')
#     master.mav.rc_channels_override_send(
#         master.target_system,  # system ID
#         master.target_component,  # component ID
#         FORWARD_CHANNEL,  # channel number
#         FORWARD_PWM, # PWM value
#         0,  # PWM value for channel 2
#         0,  # PWM value for channel 3
#         0,  # PWM value for channel 4
#         0,  # PWM value for channel 5
#         0,
#         0# PWM value for channel 6
#     )
#     print('forward')
#     set_target_yaw(TARGET_YAW)
#     # Sleep for a short period to avoid busy looping
#     time.sleep(0.1)
