"""
Example code of how to read and write vehicle parameters using pymavlink
"""

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('com4')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Loop continuously
while True:
    # Receive message
    message = master.recv_match(type='VFR_HUD ', blocking=True)
    # Convert the message to a dictionary
    message_dict = message.to_dict()
    # Print the message
    print(message_dict)

    # Wait for a short time before receiving again
    time.sleep(0.1)
