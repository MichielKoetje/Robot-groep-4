from pyax12.connection import Connection
import smbus
import time
import numpy as np
#from nep import motor
maxSpeeds = {
    45:400,
    30:350,
    15:200
}

def getMaxSpeed(angle):
    nearest_angle = None
    for threshold_angle in sorted(maxSpeeds.keys()):
        if angle <= threshold_angle:
            nearest_angle = threshold_angle
            break
    return maxSpeeds[nearest_angle]

def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
  return (read_bytes[0]<<8) | read_bytes[1]

def ReadAngle(offset = 0):
   return ((ReadRawAngle()*360/4096+offset+180)%360)-180

try:
    dynamixel_id = 21
    maxSpeed = 500
    minSpeed = 200
    DEVICE_AS5600 = 0x36 # Default device I2C address
    bus = smbus.SMBus(3)
    serial_connection = Connection(port="/dev/serial0", baudrate=57600, rpi_gpio=True)
    serial_connection.set_cw_angle_limit(dynamixel_id, 0, degrees=False)
    serial_connection.set_ccw_angle_limit(dynamixel_id, 0, degrees=False)
    startAngle = ReadAngle(5.5)
    endAngle = int(input("hoek: "))
    totalAngle = abs(endAngle-startAngle)
    direction = int(endAngle-startAngle<0)*0x400
    timeSteps = 9
    angleLeft = totalAngle
    currentSpeed = 0
    
    for x in range(timeSteps):
        currentSpeed = minSpeed-100+(100+maxSpeed-minSpeed)*x/(timeSteps-1)
        serial_connection.set_speed(dynamixel_id,direction+currentSpeed)
        a = ReadAngle(5.5)
        angleLeft = abs(endAngle - a)
        if angleLeft<max(maxSpeeds.keys()) and getMaxSpeed(angleLeft)<=currentSpeed:
            break
        print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    print()
    while angleLeft>max(maxSpeeds.keys()):
        a = ReadAngle(5.5)
        angleLeft = abs(endAngle - a)
        print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    while abs(angleLeft)>2:
        a = ReadAngle(5.5)
        angleLeft = abs(endAngle - a)
        currentSpeed = getMaxSpeed(angleLeft)
        serial_connection.set_speed(dynamixel_id,direction+getMaxSpeed(angleLeft))
        print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
        time.sleep(0.05)
    print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
            
except Exception as e:
    print(f"Error: {e}")
# Close the serial connection
serial_connection.close()