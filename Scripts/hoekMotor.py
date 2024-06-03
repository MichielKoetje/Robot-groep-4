from pyax12.connection import Connection
import smbus
import time
import numpy as np
#from nep import motor
nepMotor = False

def ReadRawAngle(): # Read angle (0-360 represented as 0-4096)
  read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
  return (read_bytes[0]<<8) | read_bytes[1]

def ReadAngle(offset = 0):
   return ((ReadRawAngle()*360/4096+offset+180)%360)-180

try:
    dynamixel_id = 21
    maxSpeed = 500
    if not nepMotor:
        DEVICE_AS5600 = 0x36 # Default device I2C address
        bus = smbus.SMBus(3)
        
        serial_connection = Connection(port="/dev/serial0", baudrate=57600, rpi_gpio=True)
        serial_connection.set_cw_angle_limit(dynamixel_id, 0, degrees=False)
        serial_connection.set_ccw_angle_limit(dynamixel_id, 0, degrees=False)
        startAngle = ReadAngle(5.5)
        endAngle = int(input("hoek: "))
        totalAngle = abs(endAngle-startAngle)
        direction = int(endAngle-startAngle<0)*0x400
        timeSteps = 20
        angleLeft = totalAngle
        currentSpeed = 0
        for x in range(timeSteps):
            if(angleLeft<=totalAngle*2/3):
                break
            currentSpeed = maxSpeed/timeSteps*(x+1)
            serial_connection.set_speed(dynamixel_id,direction+int(currentSpeed))
            a = ReadAngle(5.5)
            angleLeft = abs(endAngle - a)
            print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
            time.sleep(0.05)
        print()

        #continue when not halfway
        while (angleLeft>=totalAngle/2):
            a = ReadAngle(5.5)
            angleLeft = abs(endAngle - a)
            if(angleLeft<30):
                serial_connection.set_speed(dynamixel_id,direction+300)
                currentspeed = 300
                break
            print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")
            time.sleep(0.05)
        midAngle = angleLeft
        midSpeed = currentSpeed
        print()

        #decelerate when halfway
        while abs(angleLeft)>2:
            a = ReadAngle(5.5)
            angleLeft = endAngle - a
            speed=midSpeed*angleLeft/midAngle
            speed=max(abs(speed),300)
            currentSpeed = speed
            if(abs(currentSpeed)>maxSpeed): break
            print(speed)
            speed = direction+speed
            serial_connection.set_speed(dynamixel_id,int(speed))
            print(f"angle: {a}, speed: {speed}, angleLeft: {angleLeft}")
            time.sleep(0.05)
        serial_connection.set_speed(dynamixel_id,0)
        a = ReadAngle(5.5)
        print(f"angle: {a}, speed: {currentSpeed}, angleLeft: {angleLeft}")

    else:
        m = motor(id=1, maxSpeed=500, minSpeed=200, startAngle=90)
        endAngle = -90
        totalAngle = abs(endAngle-m.angle)
        direction = np.sign(endAngle-m.angle)
        timeSteps = 10
        angleLeft = totalAngle
        currentSpeed = 0
        for x in range(timeSteps):
            if(angleLeft<=totalAngle/2):
                break
            currentSpeed = m.maxSpeed/timeSteps*(x+1)
            m.set_speed(dynamixel_id,direction*currentSpeed)
            a = m.ReadAngle()
            angleLeft = abs(endAngle - a)
            print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            time.sleep(0.1)
        print()

        #continue when not halfway
        while (angleLeft>=totalAngle/2):
            
            a = m.ReadAngle()
            angleLeft = abs(endAngle - a)
            print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            time.sleep(0.1)

        midAngle = angleLeft
        midSpeed = currentSpeed
        print()

        #decelerate when halfway
        while abs(angleLeft)>1:
            a = m.ReadAngle()
            angleLeft = endAngle - a
            speed=midSpeed*angleLeft/midAngle
            speed=int(np.sign(speed)*max(abs(speed),200))
            currentSpeed = speed
            if(abs(currentSpeed)>maxSpeed): break
            m.set_speed(dynamixel_id,speed)
            print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            time.sleep(0.1)
        m.set_speed(dynamixel_id,0)
        print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            
except Exception as e:
    print(f"Error: {e}")
# Close the serial connection
serial_connection.close()
