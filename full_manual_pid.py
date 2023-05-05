import time
import math
from pymavlink import mavutil
from simple_pid import PID

# Define PID controllers for forward, left and right directions
forward_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)
left_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)
right_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)


# Set the output limits for the PID controllers
forward_pid.output_limits = (1500, 1700)
left_pid.output_limits = (1300, 1500)
right_pid.output_limits = (1500, 1700)
forward_pwm = 0
left_pwm = 0
right_pwm = 0

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

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

msg = master.recv_match(type='AHRS2', blocking=True)
if msg:
    yaw = msg.yaw
    fused_heading = math.degrees(yaw)

    # Adjust heading to range 0-360 degrees
    if fused_heading < 0:
        fused_heading += 360

    # Convert heading to integer degrees
    fused_heading = int(fused_heading)

# Initialize the setpoint for each controller to the current fused heading
forward_pid.setpoint = fused_heading
left_pid.setpoint = fused_heading
right_pid.setpoint = fused_heading

# Initialize the last fused heading and the elapsed time variables
last_fused_heading = fused_heading
elapsed_time = 0
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

while True:
    start_time = time.time()

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

        if elapsed_time != 0:
            # Calculate the rate of change of the fused heading
            fused_heading_rate = (fused_heading - last_fused_heading) / elapsed_time

            # Generate the PWM signals for each direction using the PID controllers
            forward_pwm = forward_pid(fused_heading_rate)
            left_pwm = left_pid(fused_heading_rate)
            right_pwm = right_pid(fused_heading_rate)

            # Set the PWM signals for each direction using the set_rc_channel_pwm function
            if fused_heading <= 171 and fused_heading >= 169:
                print('forward')
                set_rc_channel_pwm(5, forward_pwm)
            elif fused_heading < 168:
                print('Right')
                set_rc_channel_pwm(4, right_pwm)
            elif fused_heading > 172:
                print('Left')
                set_rc_channel_pwm(4, left_pwm)

        # Update the last fused heading and elapsed time variables
        last_fused_heading = fused_heading
        elapsed_time = time.time() - start_time

    time.sleep(0.01)
