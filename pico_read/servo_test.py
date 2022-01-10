from machine import PWM, Pin
import time

servo1 = PWM(Pin(28))
servo1.freq(50)

servo1.duty_u16(1650)
time.sleep(2)


for i in range(1650,7850, 100):
    servo1.duty_u16(i)
    time.sleep(0.2)
 
     
for i in range(7850,1650, -100):
    servo1.duty_u16(i)
    time.sleep(0.2)