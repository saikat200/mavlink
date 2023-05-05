
# Import mavutil
from pymavlink import mavutil
import time
# Create the connection
master = mavutil.mavlink_connection('com5')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


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
        master.target_component,             # target_com
# Loop continuously
while True:
    # Request parameter
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'SURFACE_DEPTH',
        -1
    )

    # Wait for the response
    message = None
    while message is None:
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        time.sleep(0.1)

    # Convert the value to meters
    surface_depth_cm = message.param_value
    surface_depth_m = surface_depth_cm / 100

    # Print the value in meters
    print('SURFACE_DEPTH: %.2f meters' % surface_depth_m)

    if surface_depth_m > -0.2:
        print('down')
        set_rc_channel_pwm(3, 1600)
    elif surface_depth_m > -2.1:
        print('up')
        set_rc_channel_pwm(3, 1600)
    elif surface_depth_m > -1.1:
        print('down-slow')
        set_rc_channel_pwm(3, 1550)
    elif surface_depth_m > -1.7:
        print('up_slow')
        set_rc_channel_pwm(3, 1550)


    # Wait for a short time before requesting again
    time.sleep(0.5)
