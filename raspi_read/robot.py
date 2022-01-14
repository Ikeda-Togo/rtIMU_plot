import math
import time

class Robot:
    def __init__(self, body, arm):
        self.body=body
        self.arm=arm
    def imu_deg_estimator(self, arm_deg):
        b = self.arm
        c = self.body
        A = 180-arm_deg

        a = math.sqrt((-math.cos(math.radians(A))*2*b*c)+(b*b)+(c*c))
        print(-math.cos(math.radians(150)))
        B =  math.degrees(math.asin(math.sin(math.radians(A))*b/a))

        return B, a
    
    def servo_deg_estimator(self,imu_deg):
        b = self.body
        a = self.arm
        servo_deg =imu_deg + math.degrees(math.asin(math.sin(math.radians(imu_deg))*b/a))
        print("servo deg is :",servo_deg)
        return servo_deg