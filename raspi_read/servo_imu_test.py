import pigpio
import time
import calc_angle 
import robot
import serial
import math

imu =  calc_angle.IMU()
robot = robot.Robot(128,107)
ser = serial.Serial(
    port = "/dev/ttyACM0",  #Linux
    # port = 'COM4',            #Windows
    baudrate = 57600,
    parity = serial.PARITY_NONE,
    bytesize = serial.EIGHTBITS,
    stopbits = serial.STOPBITS_ONE,
    )

gpio_pin0 = 18
gpio_pin1 = 19
 
pi = pigpio.pi()
pi.set_mode(gpio_pin0, pigpio.OUTPUT)
 
def deg_to_duty(degree):
    dc = 2.0 + (11.0-2.0)/180*(degree+90)
    return dc


# GPIO18: 50Hz、duty比7.25%
pi.hardware_PWM(gpio_pin0,50,int(deg_to_duty(21)*10000))
pi.hardware_PWM(gpio_pin1,50,int(deg_to_duty(15)*10000))
 
try :
    while True:
        if ser.in_waiting > 0:
            print('in_waiting is',ser.in_waiting)
            recv_data = ser.read(28)
            degree = imu.GetSensorData(recv_data)

            if degree >80:
                degree = 80
            elif degree <-80:
                degree = -80

            right_deg = -degree+21
            left_deg = degree+15
            pi.hardware_PWM(gpio_pin0,50,int(deg_to_duty(right_deg)*10000))
            pi.hardware_PWM(gpio_pin1,50,int(deg_to_duty(left_deg)*10000))

 
 
except KeyboardInterrupt:
    pi.set_mode(gpio_pin0,pigpio.INPUT)
    pi.set_mode(gpio_pin1,pigpio.INPUT)
    pi.stop()
    print("estimate deg :",robot.imu_deg_estimator(degree))