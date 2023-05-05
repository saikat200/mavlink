import time
import math
from pymavlink import mavutil
import keyboard


#T_HEAD= 0
# Create the connection
master = mavutil.mavlink_connection('com5')

# Wait for the heartbeat message to find the system ID
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.get_type() == 'HEARTBEAT':
        print(f"System ID: {msg.get_srcSystem()}")
        break


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
        master.target_system,  # target_system
        master.target_component,  # target_component
        *rc_channel_values)

def head_lock():
    msg = master.recv_match(type='AHRS2', blocking=True)
    yaw = msg.yaw
    fused_heading = math.degrees(yaw)

    # Adjust heading to range 0-360 degrees
    if fused_heading < 0:
        fused_heading += 360

    # Convert heading to integer degrees
    fused_heading = int(fused_heading)

    T_HEAD= fused_heading
    print(f"target Heading: {T_HEAD}")

def head_hold_forward():
    target_head = 2
    Kp = 2.0  # Proportional gain
    Ki = 0.0  # Integral gain
    Kd = 0.0  # Derivative gain
    last_error = 0
    integral_error = 0
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

        # Calculate error
        error = target_head - fused_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Calculate PID output
        proportional = Kp * error
        integral_error += error
        integral = Ki * integral_error
        derivative = Kd * (error - last_error)
        pid_output = proportional + integral + derivative
        #print(f"pid_output:{pid_output}")

        # Update last error
        last_error = error

        # Apply PID output to control the vehicle
        if pid_output > 2:
            # Turn left
           # set_rc_channel_pwm(4, 1500 - min(pid_output, 500))
            left_speed=1500 + min(pid_output, 500)
            print(f"right_speed: {left_speed}")
        elif pid_output <=2 and pid_output >=-2:
            #set_rc_channel_pwm(5, 1550)
            print("forward")
        else:
            # Turn right
            #set_rc_channel_pwm(4, 1500 + min(-pid_output, 500))
            right_speed=1500 - min(-pid_output, 500)
            print(f"left_speed: {right_speed}")

        # Forward movement
        #set_rc_channel_pwm(6, 1550)


def head_hold_left():
    target_head = T_HEAD
    Kp = 2.0  # Proportional gain
    Ki = 0.0  # Integral gain
    Kd = 0.0  # Derivative gain
    last_error = 0
    integral_error = 0
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

        # Calculate error
        error = target_head - fused_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Calculate PID output
        proportional = Kp * error
        integral_error += error
        integral = Ki * integral_error
        derivative = Kd * (error - last_error)
        pid_output = proportional + integral + derivative
        #print(f"pid_output:{pid_output}")

        # Update last error
        last_error = error

        # Apply PID output to control the vehicle
        if pid_output > 2:
            # Turn left
           # set_rc_channel_pwm(4, 1500 - min(pid_output, 500))
            left_speed=1500 + min(pid_output, 500)
            print(f"right_speed: {left_speed}")
        elif pid_output <=2 and pid_output >=-2:
            set_rc_channel_pwm(6, 1400)
            print("left")
        else:
            # Turn right
            #set_rc_channel_pwm(4, 1500 + min(-pid_output, 500))
            right_speed=1500 - min(-pid_output, 500)
            print(f"left_speed: {right_speed}")

        # Forward movement
        #set_rc_channel_pwm(6, 1550)


def head_hold_right():
    target_head = T_HEAD
    Kp = 2.0  # Proportional gain
    Ki = 0.0  # Integral gain
    Kd = 0.0  # Derivative gain
    last_error = 0
    integral_error = 0
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

        # Calculate error
        error = target_head - fused_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Calculate PID output
        proportional = Kp * error
        integral_error += error
        integral = Ki * integral_error
        derivative = Kd * (error - last_error)
        pid_output = proportional + integral + derivative
        #print(f"pid_output:{pid_output}")

        # Update last error
        last_error = error

        # Apply PID output to control the vehicle
        if pid_output > 2:
            # Turn left
           # set_rc_channel_pwm(4, 1500 - min(pid_output, 500))
            left_speed=1500 + min(pid_output, 500)
            print(f"right_speed: {left_speed}")
        elif pid_output <=2 and pid_output >=-2:
            set_rc_channel_pwm(6, 1600)
            print("right")
        else:
            # Turn right
            #set_rc_channel_pwm(4, 1500 + min(-pid_output, 500))
            right_speed=1500 - min(-pid_output, 500)
            print(f"left_speed: {right_speed}")

        # Forward movement
        #set_rc_channel_pwm(6, 1550)


def depth_hold():
    setpoint = 67
    Kp = 0.5
    Ki = 0.01
    Kd = 0.1
    integral_error = 0
    previous_error = 0
    dt = 0.1
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



while True:
    # if keyboard.is_pressed('w'):
    #     while keyboard.is_pressed('w'):
    #         head_hold_forward()
    # elif keyboard.is_pressed('f'):
    #     while keyboard.is_pressed('f'):
    #         print("left")

    head_hold_forward()




    time.sleep(0.01)

