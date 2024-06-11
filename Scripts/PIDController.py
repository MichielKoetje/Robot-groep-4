from pyax12.connection import Connection
import smbus
import time
import math
import numpy as np
import re
from simple_pid import PID
import serial

dynamixel_id_B = 5
dynamixel_id_A = 21
deadzone = 12
targetPos = [1.2,0]
DEVICE_AS5600 = 0x36  # Default device I2C address

def ReadRawAngle(bus): # Read angle (0-360 represented as 0-4096)
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    return (read_bytes[0] << 8) | read_bytes[1]

def ReadAngle(bus,offset=0):
    return ((ReadRawAngle(bus) * 360 / 4096 + offset + 180) % 360) - 180

def AnglesFromCoords(t=[1.2,0]):
    l1 = 0.812
    l2 = 0.639
    target = t
    cosTheta2 = (target[0]*target[0]+target[1]*target[1]-l1*l1-l2*l2)/(2*l1*l2)
    sinTheta2 = math.sqrt(1-cosTheta2*cosTheta2)
    theta2=-math.atan2(sinTheta2, cosTheta2)
    k1=l1+l2*cosTheta2
    k2=-l2*sinTheta2
    theta1=math.atan2(target[1], target[0])-math.atan2(k2, k1)
    global pidA
    global pidB
    pidA.setpoint=math.degrees(-theta2)
    pidB.setpoint=math.degrees(-theta1)
    pidA.output_limits = (-500,500)
    pidB.output_limits = (-600,600)

def setServoSpeed(id,speed,invert=False):
    servoSpeed = int(speed<0 ^ invert)*0x400+abs(int(speed))
    serial_connection.set_speed(id, servoSpeed)

def formatControllerOutput(rcv):
    result = re.sub(r'[^\d\s]', '', str(rcv))
    return list(map(int,result.split(" ")))

def calculate_distance(targetPos):
    return math.sqrt(targetPos[0]**2 + targetPos[1]**2)

def limit_distance(targetPos):
    distance = calculate_distance(targetPos)
    if distance < 0.3:
        return [0.3 * (targetPos[0] / distance), 0.3 * (targetPos[1] / distance)]
    elif distance > 1.4:
        return [1.4 * (targetPos[0] / distance), 1.4 * (targetPos[1] / distance)]
    else:
        return targetPos


try:
    serial_connection = Connection(port="/dev/serial0", baudrate=57600, rpi_gpio=True)
    btSerial = serial.Serial("/dev/rfcomm0", baudrate=9600, timeout=0.5)
    busB = smbus.SMBus(3)
    busA = smbus.SMBus(2)
    pidA = PID(20,0,0.2, setpoint=0)
    pidB = PID(85,0,0.35, setpoint=0)
    serial_connection.set_cw_angle_limit(dynamixel_id_B, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id_B, 0, degrees=False)
    serial_connection.set_cw_angle_limit(dynamixel_id_A, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id_A, 0, degrees=False)

    t = True
    while True:
        rcv = btSerial.readline()
        if rcv:
            if(t):
                print("aan")
                t = False
            resultlist = formatControllerOutput(rcv)
            speedUp = -(abs(resultlist[0]-512)>deadzone)*(resultlist[0]-512)
            speedLR = (abs(resultlist[1]-512)>deadzone)*(resultlist[1]-512)
            targetPos[1] = targetPos[1]+speedLR*0.0001
            targetPos[0] = targetPos[0]+speedUp*0.0001
            targetPos = limit_distance(targetPos)
            print(targetPos)
            AnglesFromCoords(targetPos)

            current_angle_A = ReadAngle(busA,5.5)
            control_A = pidA(current_angle_A)
#           print(current_angle_A)
            setServoSpeed(dynamixel_id_A, control_A)

            current_angle_B = ReadAngle(busB,333.5)
            control_B = pidB(current_angle_B)
#           print(current_angle_B)
            setServoSpeed(dynamixel_id_B, control_B,True)

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close the serial connection
    serial_connection.close()