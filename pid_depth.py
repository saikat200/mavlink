import time
import math
from pymavlink import mavutil
import keyboard
import pid_move


# Create the connection
master = mavutil.mavlink_connection('com5')

# Wait for the heartbeat message to find the system ID
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.get_type() == 'HEARTBEAT':
        print(f"System ID: {msg.get_srcSystem()}")
        break
#arm the vehicle
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


def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)


setpoint = 67
Kp = 0.5
Ki = 0.01
Kd = 0.1
integral_error = 0
previous_error = 0
dt = 0.1

while True:
    msg = master.recv_match(type='AHRS2', blocking=True)
    # Read the current depth
    depth_m = msg.altitude
    depth = abs(int(depth_m * 100))
    print(f"depth: {depth}")

    # Calculate the error
    error = setpoint - depth

    # Calculate the integral error
    integral_error = integral_error + error * dt

    # Calculate the derivative error
    derivative_error = (error - previous_error) / dt
    previous_error = error

    # Calculate the output of the PID controller
    output = Kp * error + Ki * integral_error + Kd * derivative_error

    # Clip the output to the allowable range of PWM values
    output = max(1400, min(1600, output))

    print(f"Output: {output}")


    # Set the RC channel PWM value
    #set_rc_channel_pwm(3, output)
