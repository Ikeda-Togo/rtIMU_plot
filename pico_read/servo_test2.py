from machine import PWM, Pin
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


robot = Robot(128,107)
servo1 = PWM(Pin(27)) #Left
servo2 = PWM(Pin(28)) #Right
servo1.freq(50)
servo2.freq(50)
servo1.duty_u16(4750)
servo2.duty_u16(4950)

try:
    while True :
        deg = int(input("角度入力："))
        print(robot.imu_deg_estimator(deg))
        servo1.duty_u16(4750+deg*35)
        servo2.duty_u16(4950-deg*35)
        time.sleep(2)


except KeyboardInterrupt:
    servo1.deinit()
    servo2.deinit()
    print("stop!!")