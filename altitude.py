import time
from pymavlink import mavutil

# Connect to the ArduSub autopilot
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# Wait for the first heartbeat
master.wait_heartbeat()

# Print the ArduSub autopilot type
print("Autopilot Type:", master.target_system)

def head_hold():
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
        if fused_heading <= 171 and fused_heading >= 169:
            print('forward')
            set_rc_channel_pwm(5, forward_pwm)
        elif fused_heading < 168:
            print('Right')
            set_rc_channel_pwm(4, right_pwm)
        elif fused_heading > 172:
            print('Left')
            set_rc_channel_pwm(4, left_pwm)

    time.sleep(0.01)



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
        master.target_component,
        *rc_channel_values)   # target_com

# Continuously read and print the altitude in meters using the VFR_HUD message
while True:
    try:
        # Wait for the next VFR_HUD message and extract the altitude field
        msg = master.recv_match(type='VFR_HUD', blocking=True)
        altitude_m = msg.alt
        #altitude = round(altitude_m, 2)

        # Print the altitude in meters
        print("Altitude (m):", altitude_m)

        # if altitude > -0.0 and altitude < 0.5:
        #     print('down')
        #     set_rc_channel_pwm(3, 1600)
        # # elif altitude > -0.8:
        # #     print('up')
        #     #set_rc_channel_pwm(3, 1600)
        # elif altitude >= -0.5 and altitude == 0.7:
        #     print('down-slow')
        #     set_rc_channel_pwm(3, 1550)
        # elif altitude > -0.7:
        #     print('stop')
        #     set_rc_channel_pwm(3, 1500)

    except KeyboardInterrupt:
        # Close the connection when the user interrupts the program
        print("Closing connection...")
        master.close()
        break

    except Exception as e:
        # Print any other exceptions that occur
        print(e)
