import time
import numpy as np
from nep import motor
nepMotor = False
maxSpeeds = {
    45:400,
    30:350,
    15:250
}

def getMaxSpeed(angle):
    nearest_angle = None
    for threshold_angle in sorted(maxSpeeds.keys()):
        if angle <= threshold_angle:
            nearest_angle = threshold_angle
            break
    return maxSpeeds[nearest_angle]
try:
    dynamixel_id = 21
    maxSpeed = 500
    m = motor(id=1, maxSpeed=500, minSpeed=200, startAngle=-15)
    endAngle = 15
    totalAngle = abs(endAngle-m.angle)
    direction = np.sign(endAngle-m.angle)
    timeSteps = 9
    angleLeft = totalAngle
    currentSpeed = 0
    
    for x in range(timeSteps):
        currentSpeed = m.minSpeed-100+(100+m.maxSpeed-m.minSpeed)*x/(timeSteps-1)
        m.set_speed(dynamixel_id,direction*currentSpeed)
        a = m.ReadAngle()
        angleLeft = abs(endAngle - a)
        if angleLeft<max(maxSpeeds.keys()) and getMaxSpeed(angleLeft)<=currentSpeed:
            break
        print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    print()
    while angleLeft>max(maxSpeeds.keys()):
        a = m.ReadAngle()
        angleLeft = abs(endAngle - a)
        print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    while abs(angleLeft)>2:
        a = m.angle
        angleLeft = abs(endAngle - a)
        m.set_speed(dynamixel_id,direction*getMaxSpeed(angleLeft))
        m.ReadAngle()
        print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            
except Exception as e:
    print(f"Error: {e}")
# Close the serial connection
#serial_connection.close()