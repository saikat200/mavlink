import time
from pymavlink import mavutil
import keyboard
import math
fixed_value = None  # Initialize fixed value to None

# Set up the connection to the ArduSub vehicle
master = mavutil.mavlink_connection('com4')


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



# master.arducopter_arm() or:
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

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

# Example usage: move forward at speed 500 for 2 seconds
#go_forward(600)
#time.sleep(5)
#stop()
#time.sleep(5)
# go_forward(600)
# time.sleep(5)
# stop()
while True:
    # Get AHRS2 message
    msg = master.recv_match(type='AHRS2', blocking=True)
    if msg:
        yaw = msg.yaw
        fused_heading = math.degrees(yaw)

        # Adjust heading to range 0-360 degrees
        if fused_heading < 0:
            fused_heading += 360

        # Convert heading to integer degrees
        fused_heading = int(fused_heading)
        print(f"Fused Heading: {fused_heading}")

        # Check if 'a' key is pressed to fix the current value of fused_heading
        if keyboard.is_pressed('a'):
            fixed_value = fused_heading
            print(f"Fixed value: {fixed_value}")
        elif keyboard.is_pressed('x'):
            stop()
            print('stop')
            break

        # Check if fixed_value has been set
        elif fixed_value is not None:
            # Check if fused_heading is lower than fixed_value
            if fused_heading >= fixed_value - 3 and fused_heading <= fixed_value + 3:
                print('forward')
                go_forward(600)
            elif fused_heading < fixed_value:
                print('left')
                go_lateral_left(600)
            # Check if fused_heading is higher than fixed_value
            elif fused_heading > fixed_value:
                print('right')
                go_lateral_right(600)

            # If fused_heading equals fixed_value


    time.sleep(0.01)