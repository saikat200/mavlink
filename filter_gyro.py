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

# Set filter coefficients
fc = 1  # Cutoff frequency in Hz
dt = 0.1  # Sampling time in seconds
alpha = dt / (2 * 3.14 * fc + dt)

# Initialize filtered values
filtered_ax = 0
filtered_ay = 0
filtered_az = 0
filtered_gx = 0
filtered_gy = 0
filtered_gz = 0

# Loop continuously
while True:
    # Receive message
    message = master.recv_match(type='SCALED_IMU2', blocking=True)
    # Convert the message to a dictionary
    message_dict = message.to_dict()

    # Apply low-pass filter to acceleration and gyro data
    filtered_ax = (1 - alpha) * filtered_ax + alpha * message_dict['xacc']
    filtered_ay = (1 - alpha) * filtered_ay + alpha * message_dict['yacc']
    filtered_az = (1 - alpha) * filtered_az + alpha * message_dict['zacc']
    filtered_gx = (1 - alpha) * filtered_gx + alpha * message_dict['xgyro']
    filtered_gy = (1 - alpha) * filtered_gy + alpha * message_dict['ygyro']
    filtered_gz = (1 - alpha) * filtered_gz + alpha * message_dict['zgyro']

    # Print the filtered values

    print('Filtered gyro: (%.2f, %.2f, %.2f)' % (int(filtered_gx), int(filtered_gy), int(filtered_gz)))

    # Wait for a short time before receiving again
    time.sleep(dt)