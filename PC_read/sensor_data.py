import serial
import struct
import time
import pandas as pd
import matplotlib.pyplot as plt

class IMU:
    def __init__(self):#constructor

        self.ser = serial.Serial(
            port = "/dev/ttyACM0",
            baudrate = 115200,
            #parity = serial.PARITY_NONE,
            bytesize = serial.EIGHTBITS,
            stopbits = serial.STOPBITS_ONE,
            #timeout = 0.01,
            #xonxoff = 0,
            #rtscts = 0,
            )

        self.count = 0
        self.ut = time.time()
        self.pre_time_stamp = 0
        self.pregy = 0

        self.acc=[0] * 3
        self.gyro = [0] * 3
        self.degree = 0

#バイナリデータの上位と下位を計算
    def BinaryCalc (self ,low_bit, high_bit):
        Low_data=low_bit
        High_data=high_bit << 8
        Data =Low_data + High_data

        if Data >= 32767:
            Data = Data -65535

        return Data
    
    def get_gyro_degree(self,gy,dt):
        self.degree += (self.pregy + gy) * dt / 2
        self.pregy = gy
        return self.degree

    def get_acc_degree(self,acc):
        if acc[0] > 1:
            acc[0] = 1
        if acc[2] > 2:
            acc[2] = 2
        elif acc[2] < 0:
            acc[2] = 0
        


        if 0 <= acc[0] <= 1 and 0 <= acc[2] <= 1:
            deg = 90 * acc[0]

        elif 0 <= acc[0] <= 1 and 1 < acc[2] <= 2:
            deg = 180 - 90 * acc[0]
        
        elif -1 <= acc[0] < 0 and 0 <= acc[2] <= 1:
            deg = 90 * acc[0]

        elif acc[0] < -1 and 0 <= acc[2]:
            deg = 90 * acc[0]

        elif -1 <= acc[0] < 0 and 1 < acc[2] <= 2:
            deg = -180 - 90 * acc[0]    
        else :
            print("OUT RANGE")
            deg = 0
        return deg
    
    def GetSensorData(self):

        if self.ser.in_waiting > 0:

            time_stamp = time.time() - self.ut
            dt=time_stamp - self.pre_time_stamp
            self.pre_time_stamp=time_stamp
        
            recv_data = self.ser.read(28)


            self.acc[0] =self.BinaryCalc(recv_data[8],recv_data[9])
            self.acc[1] =self.BinaryCalc(recv_data[10],recv_data[11])
            self.acc[2] =self.BinaryCalc(recv_data[12],recv_data[13])
            self.gyro[0] = self.BinaryCalc(recv_data[16],recv_data[17])
            self.gyro[1] = self.BinaryCalc(recv_data[18],recv_data[19])
            self.gyro[2] = self.BinaryCalc(recv_data[20],recv_data[21])


            self.acc[0] = self.acc[0]/2048
            self.acc[1] = self.acc[1]/2048
            self.acc[2] = self.acc[2]/2048

            self.gyro[0] = self.gyro[0]/16.4
            self.gyro[1] = self.gyro[1]/16.4
            self.gyro[2] = self.gyro[2]/16.4


            gyro_deg = self.get_gyro_degree(self.gyro[1],dt)
            acc_deg = self.get_acc_degree(self.acc)
            

            print("Time stamp:",time_stamp)
            print("dt:",dt)
            print("data type :",type(recv_data))
            print("recv raw data:",recv_data )
            print("------------------------------------")
            print("X acc is :",self.acc[0] )
            print("Y acc is :",self.acc[1] )
            print("Z acc is :",self.acc[2] )
            print("X gyro is :",self.gyro[0] )
            print("Y gyro is :",self.gyro[1] )
            print("Z gyro is :",self.gyro[2] )
            print("------------------------------------")
            print("gyro deg:",gyro_deg)
            print("acc deg:",acc_deg) 
            print(" ")
        
        else :
            acc_deg = 0
            gyro_deg = 0
            time_stamp = 0
            # if acc_deg == 0:
            #     time.sleep(10)

        return acc_deg,gyro_deg,time_stamp
    
