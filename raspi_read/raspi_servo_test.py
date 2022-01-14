import RPi.GPIO as GPIO
import time

pin = [17, 18] 
servo = []
GPIO.setmode(GPIO.BCM)

for num in range(2):
    GPIO.setup(pin[num], GPIO.OUT)
    servo.append(GPIO.PWM(pin[num], 50))
    servo[num].start(6.5)

time.sleep(1.0)

try:
    while True:
        print("input Degree(-90~90):")
        degree = int(input())
        right_deg = degree+15
        left_deg = -degree
        right_dc = 2.0 + (11.0-2.0)/180*(right_deg+90)
        left_dc = 2.0 + (11.0-2.0)/180*(left_deg+90)
        servo[0].ChangeDutyCycle(right_dc)
        servo[1].ChangeDutyCycle(left_dc)
        print("degree is",right_deg,left_deg)
except KeyboardInterrupt:
    for num in range(2):
        servo[num].stop()
    GPIO.cleanup()
