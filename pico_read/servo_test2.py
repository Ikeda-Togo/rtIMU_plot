from machine import PWM, Pin
import time

servo1 = PWM(Pin(28))
servo1.freq(50)

max_duty = 65025
dig_0 = 0.0725    #0°
dig_90 = 0.12     #90°

while True:
    servo1.duty_u16(int(max_duty*dig_0))
    servo1.duty_u16(int(max_duty*dig_90))
