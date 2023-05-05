import time
import threading
from pymavlink import mavutil

# Set up the connection to the ArduSub vehicle
master = mavutil.mavlink_connection('com5')


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

def handle_input():
    while True:
        inp = input()
        if inp == 'w':
            go_forward(600)
            print('forward')
        elif inp == 's':
            go_backward(600)
            print('backward')
        elif inp == 'a':
            go_lateral_left(600)
        elif inp == 'd':
            go_lateral_right(600)
        elif inp == 'q':
            yaw_left(600)
        elif inp == 'e':
            yaw_right(600)
        elif inp == ' ':
            stop()
            print('stop')

# Start a thread to handle user input
input_thread = threading.Thread(target=handle_input)
input_thread.daemon = True
input_thread.start()

# Wait forever so the input thread continues running
while True:
    time.sleep(1)
