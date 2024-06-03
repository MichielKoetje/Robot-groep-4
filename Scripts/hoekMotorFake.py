import time
import numpy as np
from nep import motor
nepMotor = False
maxSpeeds = {
    45:400,
    30:350,
    15:250
}
try:
    dynamixel_id = 21
    maxSpeed = 500
    m = motor(id=1, maxSpeed=500, minSpeed=200, startAngle=45)
    endAngle = 0
    totalAngle = abs(endAngle-m.angle)
    direction = np.sign(endAngle-m.angle)
    timeSteps = 10
    angleLeft = totalAngle
    currentSpeed = 0
    print(maxSpeeds[max(maxSpeeds)])
    for x in range(timeSteps):
        if(angleLeft<=totalAngle/2):
            break
        currentSpeed = m.minspeed-100+(100+m.maxspeed-m.minspeed)*x/(timeSteps-1)
        m.setSpeed()
        a = m.ReadAngle()
        angleLeft = abs(endAngle - a)
        print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
        time.sleep(0.1)
    print()
    print(f"angle: {a}, speed: {m.speed}, angleLeft: {angleLeft}")
            
except Exception as e:
    print(f"Error: {e}")
# Close the serial connection
#serial_connection.close()