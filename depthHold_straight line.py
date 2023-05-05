import time
from pymavlink import mavutil

# Connect to the vehicle (replace with appropriate connection string)
vehicle = mavutil.mavlink_connection('com4')

# Arm the vehicle
vehicle.arducopter_arm()
print('arm')

# Set the mode to depth hold
vehicle.set_mode('GUIDED', custom_mode=10) # 10 is the custom mode for depth hold

# Set the target altitude in feet
target_altitude = 10 # replace with desired altitude in feet
vehicle.mav.set_position_target_global_int_send(
    int(1e7), int(1e7), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)

# Move forward in a straight line
vehicle.mav.set_position_target_global_int_send(
    int(1e7), int(1e7 + 50), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(1) # replace with desired duration

# Maintain heading and depth while moving forward
vehicle.mav.set_position_target_global_int_send(
    int(1e7), int(1e7 + 50), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)
vehicle.mav.set_attitude_target_send(
    0, 0, 0, 0, 0, 0, 0, 0)
print('forward')


# Add lateral movement with 'a' and 'd' keys
while True:
    key = input() # get input from user
    if key == 'a':
        # move laterally left
        vehicle.mav.set_position_target_global_int_send(
            int(1e7 - 50), int(1e7), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)
        print('left')
    elif key == 'd':
        # move laterally right
        vehicle.mav.set_position_target_global_int_send(
            int(1e7 + 50), int(1e7), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)
        print('right')
    else:
        # stop lateral movement
        vehicle.mav.set_position_target_global_int_send(
            int(1e7), int(1e7), target_altitude * 100, 0, 0, 0, 0, 0, 0, 0, 0)
