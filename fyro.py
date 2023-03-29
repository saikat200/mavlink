import time
from pymavlink import mavutil

# Set up a MAVLink connection to the vehicle
connection_string = 'com4'  # Replace with your connection string
master = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message to ensure a connection is established
master.wait_heartbeat()

# Request IMU data at 100 Hz
master.mav.request_data_stream_send(
    master.target_system,  # Target system ID
    master.target_component,  # Target component ID
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # Data stream ID
    100,  # Requested message rate (Hz)
    1  # Enable message sending
)

# Initialize the filter parameters
alpha = 0.5
gyro_x_filtered = 0.0
gyro_y_filtered = 0.0
gyro_z_filtered = 0.0

# Continuously read and filter the gyro data
while True:
    msg = master.recv_match(type='SCALED_IMU2', blocking=True)
    if msg:
        # Apply the low-pass filter to the gyro data
        gyro_x_filtered = alpha * gyro_x_filtered + (1 - alpha) * msg.xgyro
        gyro_y_filtered = alpha * gyro_y_filtered + (1 - alpha) * msg.ygyro
        gyro_z_filtered = alpha * gyro_z_filtered + (1 - alpha) * msg.zgyro
        print('Gyro X:', int(gyro_x_filtered), 'Gyro Y:', gyro_y_filtered, 'Gyro Z:', int(gyro_z_filtered))
