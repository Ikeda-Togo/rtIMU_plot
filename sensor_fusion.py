import calc_angle
import serial
import pandas as pd
import matplotlib.pyplot as plt
import time

imu = calc_angle.IMU()

ser = serial.Serial(
    port = "/dev/ttyACM0",
    baudrate = 115200,
    #parity = serial.PARITY_NONE,
    bytesize = serial.EIGHTBITS,
    stopbits = serial.STOPBITS_ONE,
    # timeout = 0.01,
    #xonxoff = 0,
    #rtscts = 0,
    )

file = open("angle_data.csv", "w")

try:
    while(True):
        if ser.in_waiting > 0:
            recv_data = ser.read(28)
            time_stamp,acc_pitch,gyro_pitch,filter_pitch=imu.GetSensorData(recv_data)
            file.write(str(time_stamp) + "," + str(acc_pitch) + "," + str(gyro_pitch) + "," + str(filter_pitch) + "\n")  


except KeyboardInterrupt:
    file.close()
    
    df = pd.read_csv('angle_data.csv', names=['num1', 'num2','num3','num4'])
    # plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
    plt.figure()
    plt.plot(df['num1'],df['num2'],label='acc_angle')
    plt.plot(df['num1'],df['num3'],label= "gyro_angle")
    plt.plot(df['num1'],df['num4'],label= "filter_angle")
    plt.legend()
    plt.show()
