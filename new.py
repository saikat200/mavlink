import time
import math
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('com5')

# Wait for the heartbeat message to find the system ID
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.get_type() == 'HEARTBEAT':
        print(f"System ID: {msg.get_srcSystem()}")
        break

# Set mode to stabilized
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# Wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Initialize PID controller

last_error = 0
error_sum = 0

# Set initial PWM values
forward_pwm = 1590
left_pwm = 1500
right_pwm = 1590



def pid_control(desired_heading, current_heading):
    global last_error, error_sum, forward_pwm, left_pwm, right_pwm

    # Calculate error
    error = desired_heading - current_heading

    # Calculate error sum
    error_sum += error

    # Calculate error derivative
    error_deriv = error - last_error
    last_error = error

    # Calculate output using PID equation
    kp = 1.0
    ki = 0.1
    kd = 0.2
    output = kp * error + ki * error_sum + kd * error_deriv

    # Apply saturation limits
    output = max(min(output, 200), -200)

    # Apply deadband
    if abs(error) < 5:
        output = 0

    # Set forward PWM value
    # forward_pwm = 1600 + int(output)
    # forward_pwm = max(min(forward_pwm, 1700), 1500)
    forward_pwm=1600


    # Set left and right PWM values
    if error > 0:
        right_pwm = 1500 + int(output)
        left_pwm = 1500
        right_pwm = max(min(right_pwm, 1700), 1300)

    elif error < 0:
        left_pwm = 1500 - int(output)
        right_pwm = 1500
        left_pwm = max(min(left_pwm, 1500), 1300)

    else:
        left_pwm = 1500
        right_pwm = 1500

    # Set PWM values
    set_rc_channel_pwm(4, right_pwm)
    set_rc_channel_pwm(5, forward_pwm)
    set_rc_channel_pwm(6, left_pwm)

    # Print current PWM values and heading
    print(f"Current PWM values: Forward={forward_pwm}, Left={left_pwm}, Right={right_pwm}")
    print(f"Fused Heading: {current_heading}")

# Define PID controller function
# def pid_control(desired_heading, current_heading):
#     global last_error, error_sum, forward_pwm, left_pwm, right_pwm
#
#     # Calculate error
#     error = desired_heading - current_heading
#     #print(error)
#
#     # Calculate error sum
#     error_sum += error
#
#     # Calculate error derivative
#     error_deriv = error - last_error
#     last_error = error
#
#     # Calculate output using PID equation
#     output = kp * error + ki * error_sum + kd * error_deriv
#    # print(int(output))
#
#     # Set forward PWM value
#     forward_pwm = 1600 + int(output)
#     if forward_pwm < 1300:
#         forward_pwm = 1300
#     elif forward_pwm > 1700:
#         forward_pwm = 1700
#
#     # Set left and right PWM values
#     if error > 0:
#         right_pwm = 1500 + int(output)
#         left_pwm = 1500 + int(output)
#         if right_pwm > 1700:
#             right_pwm = 1700
#         if right_pwm < 1300:
#             right_pwm = 1300
#     elif error < 0:
#         left_pwm = 1500 - int(output)
#         right_pwm = 1500 - int(output)
#         if left_pwm < 1300:
#             left_pwm = 1300
#
#
#     # Set PWM values
#     set_rc_channel_pwm(4, right_pwm)
#     set_rc_channel_pwm(5, forward_pwm)
#     set_rc_channel_pwm(6, left_pwm)
#
#     # Print current PWM values and heading
#     print(f"Current PWM values: Forward={forward_pwm}, Left={left_pwm}, Right={right_pwm}")
#     print(f"Fused Heading: {current_heading}")


# Define function to set RC channel PWM value
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
        *rc_channel_values)                  # RC channel list, in microseconds.
#my_desire = range(147, 157)
desire=210
while True:
    # Get AHRS2 message
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg:
        yaw = msg.yaw
        fused_heading = math.degrees(yaw)

        # Adjust heading to range 0-360 degrees
        if fused_heading < 0:
            fused_heading += 360

        # Convert heading to integer degrees
        fused_heading = int(fused_heading)
        print(f"Fused Heading: {fused_heading}")
        #for desire in my_desire:
        pid_control(desire,fused_heading)





    time.sleep(0.01)