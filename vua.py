import time
import math
from pymavlink import mavutil
# print(mavutil.mavlink_version())
import keyboard

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
# Set mode to stabilized



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
        *rc_channel_values[:8],  # channel 1-8 values
        0, 0, 0, 0, 0, 0, 0, 0)


def head_hold():
    target_head=260
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg:
        yaw = msg.yaw
        fused_heading = math.degrees(yaw)

        # Adjust heading to range 0-360 degrees
        if fused_heading < 0:
            fused_heading += 360

        # Convert heading to integer degrees
        fused_heading = int(fused_heading)



        # Print the fused heading
        print(f"Fused Heading: {fused_heading}")
        err_head= abs((target_head-fused_heading)*2)
        if fused_heading >= target_head and fused_heading <= target_head+3:
            print('forward')
            set_rc_channel_pwm(6, 1550)
        elif fused_heading < target_head:
            print('Right')
            set_rc_channel_pwm(4, 1500+err_head)
        elif fused_heading > target_head:
            print('Left')
            set_rc_channel_pwm(4, 1500-err_head)

    time.sleep(0.01)

while True:
    # Get AHRS2 message
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg:
        depth = msg.altitude
        depth_m=abs(int(depth*100))
        depth_l=round(depth_m,2)
        t_alt = 67
        c_altitude = abs(int(depth*100))
        err = abs((t_alt - c_altitude) * 2)

        if c_altitude < t_alt:
            set_rc_channel_pwm(3, 1600 + err)
        elif c_altitude == t_alt:
            set_rc_channel_pwm(3, 1500)
        else:
            set_rc_channel_pwm(3, 1400 - err)




        # Print the fused heading
        print(f"depth Heading: {depth_m}")
        head_hold()

    time.sleep(0.01)
