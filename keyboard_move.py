import time
import keyboard
from pymavlink import mavutil

master = mavutil.mavlink_connection('com4')
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.get_type() == 'HEARTBEAT':
        print(f"System ID: {msg.get_srcSystem()}")
        break
# Set up the connection to the ArduSub vehicle



# Arm
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()
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

def move_vehicle(x, y, z, r):
    # x,y,r will be between [-1000 and 1000]
    # z will work between [0-1000]
    # Warning: z 0 is full reverse, 500 is no output and 1000 is full throttle.
    x = int(max(-1000, min(x, 1000)))
    y = int(max(-1000, min(y, 1000)))
    z = int(max(0, min(z, 1000)))
    r = int(max(-1000, min(r, 1000)))
    master.mav.manual_control_send(master.target_system, x, y, z, r, 0)

def go_forward(speed):
    move_vehicle(speed, 0, 500, 0)

def go_backward(speed):
    move_vehicle(-speed, 0, 500, 0)
def go_lateral_left(speed):
    move_vehicle(0, -speed, 500, 0)

def go_lateral_right(speed):
    move_vehicle(0, speed, 500, 0)

def yaw_left(speed):
    move_vehicle(0, 0, 500, -speed)

def yaw_right(speed):
    move_vehicle(0, 0, 500, speed)

def stop():
    move_vehicle(0, 0, 500, 0)

# The code for going left, right, yaw left, and yaw right is removed as they are not needed for this example.

# Wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Use a loop to continually check for keyboard input
while True:
    # If 'w' is pressed, move forward at speed 600
    if keyboard.is_pressed('w'):
        go_forward(600)
        print('forward')
    # If 's' is pressed, move backward at speed 600
    elif keyboard.is_pressed('s'):
        go_backward(600)
        print('backward')
    # If 'a' is pressed, move left at speed 600 (you can modify this function to move right)
    elif keyboard.is_pressed('a'):
        go_lateral_left(600)
    # If 'd' is pressed, move right at speed 600 (you can modify this function to move left)
    elif keyboard.is_pressed('d'):
        go_lateral_right(600)
    # If 'space' is pressed, stop the vehicle
    elif keyboard.is_pressed('space'):
        stop()
        print('stop')
    # If 'q' is pressed, exit the loop and disarm the vehicle
    elif keyboard.is_pressed('q'):
        stop()
        time.sleep(1)  # wait for the vehicle to stop
        # Disarm the vehicle
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        # Wait until disarming confirmed
        master.motors_disarmed_wait()
        break