import math
class motor: 
    def __init__(self,id,maxSpeed,minSpeed,startAngle):
        self.id = id
        self.maxSpeed = maxSpeed
        self.minSpeed = minSpeed
        self.speed = 0
        self.angle = startAngle
    def set_speed(self,id,speed):
        self.speed = speed
    def ReadAngle(self):
        self.angle += self.speed*abs(self.speed)*abs(self.speed)/pow(2,24)
        self.angle = round(self.angle,2)
        return self.angle