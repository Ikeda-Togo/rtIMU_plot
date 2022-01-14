import time

#GPIOの初期設定
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#GPIO4を出力端子設定
GPIO.setup(17, GPIO.OUT)

#GPIO4をPWM設定、周波数は50Hz
p = GPIO.PWM(17, 50)

#Duty Cycle 0%
p.start(6.5)

while True:
    print("input Degree : ")
    degree = float(input())
    dc =  2.0 + (11.0-2.0)/180*(degree+90)
    p.ChangeDutyCycle(dc)
    time.sleep(0.03)
