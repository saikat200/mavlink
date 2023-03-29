# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('com4')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Set the target system and component IDs
sysid = 1
compid = 1

# Set the servo number to control (assuming it is connected to the Pixhawk AUX1 port)
servo_num = 1

# Set the PWM values for the servo positions (assuming a 2000us PWM range)
center_pwm = 1500
rotated_pwm = 1900



 #Arm
# master.arducopter_arm() or:
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
# Send the command to rotate the servo to the desired position
master.mav.command_long_send(
    sysid, compid,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,            # servo number
    rotated_pwm,  # PWM value
    0, 0, 0, 0, 0, 0 # parameters
)

# Wait for 5 seconds
time.sleep(5)

# Send the command to return the servo to the center position
master.mav.command_long_send(
    sysid, compid,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,            # servo number
    center_pwm,   # PWM value
    0, 0, 0, 0, 0, 0 # parameters
)
