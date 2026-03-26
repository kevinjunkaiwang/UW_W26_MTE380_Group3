from serial_communication import SerialCommunication

robot = SerialCommunication(port="/dev/ttyACM0")

try:
    robot.teleop()
finally:
    robot.disconnect()