from pymavlink import mavutil
import time

# Set the channel number for the forward thruster
FORWARD_CHANNEL = 3

# Connect to the vehicle
master = mavutil.mavlink_connection('com4', baud=115200)

# Set the vehicle to Stabilize mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mavutil.mavlink.MAV_MODE_STABILIZE_DISARMED
)
print('stabilized')
# Set the target yaw
msg = master.mav.set_attitude_target_encode(
    0,  # time_boot_ms
    master.target_system,  # target system
    master.target_component,  # target component
    0b00001000,  # type mask: ignore roll and pitch, use yaw
    [0.0, 0.0, 0.0],  # q: attitude quaternion (not used)
    [0.0, 0.0, 0.0],  # body roll rate, pitch rate, yaw rate
    0.0,  # thrust
    0.0,  # roll speed (not used)
    0.0,  # pitch speed (not used)

)

# Disable the forward thruster by setting the thrust value to 0
msg = master.mav.set_attitude_target_encode(
    0,  # time_boot_ms
    master.target_system,  # target system
    master.target_component,  # target component
    0b00010000,  # type mask: ignore yaw, use body roll and pitch
    [0.0, 0.0, 0.0],  # q: attitude quaternion (not used)
    [0.0, 0.0, 0.0],  # body roll rate, pitch rate, yaw rate (not used)
    0.0,  # thrust
    0.0,  # roll speed (not used)
    0.0,  # pitch speed (not used)

)

# Wait for a short period to allow the vehicle to respond to the changes
time.sleep(0.1)
