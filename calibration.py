import time
from pymavlink import mavutil

# Connect to the MAVLink device
master = mavutil.mavlink_connection('com4')

# Send a command to the device to start calibration
master.mav.command_long_send(
    master.target_system,     # Target system ID
    mavutil.mavlink.MAV_COMP_ID_ALL,     # Target component ID
    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,    # Command ID
    0,    # Confirmation
    1,    # Gyro calibration
    0,    # Magnetometer calibration
    0,    # Level horizon calibration
    0,    # Magnetometer calibration quality check
    0,    # Accelerometer calibration
    0,    # Compass/Motor interference calibration
    0     # Use onboard hysteresis
)

# Wait for the calibration to complete
while True:
    msg = master.recv_match(type='CAL_STATUS', blocking=True)
    if msg:
        if msg.cal_status == 3:  # Calibration complete
            break
    time.sleep(0.1)

# Get the calibration offsets
gyro_offset_x = master.param_get('IMU_GYROXOFF')
gyro_offset_y = master.param_get('IMU_GYROYOFF')
gyro_offset_z = master.param_get('IMU_GYROZOFF')

accel_offset_x = master.param_get('IMU_ACCELXOFF')
accel_offset_y = master.param_get('IMU_ACCELYOFF')
accel_offset_z = master.param_get('IMU_ACCELZOFF')

mag_offset_x = master.param_get('COMPASS_OFS_X')
mag_offset_y = master.param_get('COMPASS_OFS_Y')
mag_offset_z = master.param_get('COMPASS_OFS_Z')

print("Gyro offsets: ", gyro_offset_x, gyro_offset_y, gyro_offset_z)
print("Accelerometer offsets: ", accel_offset_x, accel_offset_y, accel_offset_z)
print("Magnetometer offsets: ", mag_offset_x, mag_offset_y, mag_offset_z)
