import msvcrt
import time
from pymavlink import mavutil

# Set up the connection to the autopilot
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()

# Set mode to Stabilize
mode = 'STABILIZE'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

desired_heading = None
current_heading = None

while True:
    # Get the current heading
    heading_msg = master.recv_match(type='AHRS2', blocking=False)
    if heading_msg:
        current_heading = heading_msg.yaw

    # Print current heading
    print(f"Current heading: {current_heading:.2f}")

    # Check for key presses
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'w':
            desired_heading = current_heading
            print(f"Desired heading set to {desired_heading:.2f}")

    if desired_heading is not None:
        # Compute the error between current heading and desired heading
        error = desired_heading - current_heading

        # Compute the roll angle required to maintain the desired yaw heading
        roll_angle = max(-45, min(45, error * 0.5))

        # Set roll angle and throttle to control the vehicle
        throttle = 700
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'a':
                master.mav.manual_control_send(
                    master.target_system,  # target_system
                    1500,  # x-axis: no movement
                    -700,  # y-axis: forward
                    0,  # z-axis: no movement
                    0,  # r-axis: no movement
                    0)  # buttons: no button pressed
            elif key == b'b':
                master.mav.manual_control_send(
                    master.target_system,  # target_system
                    700,  # x-axis: lateral left
                    0,  # y-axis: no movement
                    0,  # z-axis: no movement
                    0,  # r-axis: no movement
                    0)  # buttons: no button pressed
            elif key == b'c':
                master.mav.manual_control_send(
                    master.target_system,  # target_system
                    -700,  # x-axis: lateral right
                    0,  # y-axis: no movement
                    0,  # z-axis: no movement
                    0,  # r-axis: no movement
                    0)  # buttons: no button pressed
            else:
                master.mav.manual_control_send(
                    master.target_system,  # target_system
                    0,  # x-axis: no movement
                    700,  # y-axis: forward
                    roll_angle,  # z-axis: roll angle to maintain yaw heading
                    0,  # r-axis: no movement
                    0)  # buttons: no button pressed

    time.sleep(0.1)
