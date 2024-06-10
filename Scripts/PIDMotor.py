from pyax12.connection import Connection
import smbus
import time
import math
import numpy as np
from simple_pid import PID

def ReadRawAngle(bus): # Read angle (0-360 represented as 0-4096)
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    return (read_bytes[0] << 8) | read_bytes[1]

def ReadAngle(bus,offset=0):
    return ((ReadRawAngle(bus) * 360 / 4096 + offset + 180) % 360) - 180

try:
    dynamixel_id_B = 5
    dynamixel_id_A = 21
    DEVICE_AS5600 = 0x36  # Default device I2C address
    busB = smbus.SMBus(3)
    busA = smbus.SMBus(2)
    serial_connection = Connection(port="/dev/serial0", baudrate=57600, rpi_gpio=True)
    serial_connection.set_cw_angle_limit(dynamixel_id_B, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id_B, 0, degrees=False)
    serial_connection.set_cw_angle_limit(dynamixel_id_A, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id_A, 0, degrees=False)

    l1 = 0.812
    l2 = 0.639
    target = (1.4,0)
    cosTheta2 = (target[0]*target[0]+target[1]*target[1]-l1*l1-l2*l2)/(2*l1*l2)
    sinTheta2 = math.sqrt(1-cosTheta2*cosTheta2)
    theta2=-math.atan2(sinTheta2, cosTheta2)
    k1=l1+l2*cosTheta2
    k2=-l2*sinTheta2
    theta1=math.atan2(target[1], target[0])-math.atan2(k2, k1)

    # PID controller setup
    pidA = PID(20,0,0.2, setpoint=math.degrees(-theta2))
    pidB = PID(85,0,0.35, setpoint=math.degrees(-theta1))
    pidA.output_limits = (-500,500)  # Output limits for motor speed (example limits)
    pidB.output_limits = (-600,600)  # Output limits for motor speed (example limits)

    while True:
        current_angle_A = ReadAngle(busA,5.5)
        control_A = pidA(current_angle_A)
        print(current_angle_A)
#        print(control_A)
        serial_connection.set_speed(dynamixel_id_A, int(control_A<0)*0x400+abs(int(control_A)))

        current_angle_B = ReadAngle(busB,333.5)
        control_B = pidB(current_angle_B)
        print(current_angle_B)
#        print(control_B)
        serial_connection.set_speed(dynamixel_id_B, int(control_B>0)*0x400+abs(int(control_B)))

        # Break condition to stop the loop
        #if abs(current_angle - end_angle) < 1:  # Tolerance of 1 degree
           # break

        time.sleep(0.1)  # Small delay to avoid excessive processing

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close the serial connection
    serial_connection.close()



#arm: pid = PID(20,0,0.2, setpoint=end_angle)
#base: pid = PID(85,0,0.35, setpoint=end_angle)
