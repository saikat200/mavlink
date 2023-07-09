from pymavlink import mavutil

# Create a MAVLink connection
mavlink_connection = mavutil.mavlink_connection('com4', baud=57600)  # Replace '/dev/ttyUSB0' with the appropriate serial port

# Wait for the first heartbeat message to ensure communication is established
mavlink_connection.wait_heartbeat()

# Main loop
while True:
    # Receive a MAVLink message
    message = mavlink_connection.recv_match(type='BATTERY_STATUS', blocking=True)

    # Check if the received message is not None
    if message is not None:
        # Extract battery status data from the message
        voltage = message.voltages[0]  # Battery voltage in millivolts
        current = message.current_battery  # Battery current in centiamperes
        remaining_capacity = message.current_consumed  # Battery consumed capacity in milliampere-hours

        # Print the battery status data
        print("Voltage: {} V".format(int(voltage / 1000.0)))
        # print("Current: {} A".format(current / 100.0))
        # print("Remaining Capacity: {} mAh".format(remaining_capacity))

# Close the MAVLink connection
mavlink_connection.close()
