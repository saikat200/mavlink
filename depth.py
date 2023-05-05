
# Import mavutil
from pymavlink import mavutil
import time
# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

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

    # Wait for a short time before requesting again
    time.sleep(0.5)
