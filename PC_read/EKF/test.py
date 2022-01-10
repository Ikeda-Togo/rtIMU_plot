import serial
import struct
import time
import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np
from ekflib import *

class IMU:
    def __init__(self):#constructor

        self.count = 0
        self.ut = time.time()
        self.pre_time_stamp = 0


        self.acc=[0] * 3
        self.gyro = [0] * 3

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.pregx = 0
        self.pregy = 0
        self.pregz = 0

        self.pre_filter_roll =0
        self.pre_filter_pitch=0
        self.pre_filter_yaw=0

        
        self.degree = 0

#バイナリデータの上位と下位を計算
    def BinaryCalc (self ,low_bit, high_bit):
        Low_data=low_bit
        High_data=high_bit << 8
        Data =Low_data + High_data

        if Data >= 32767:
            Data = Data -65535

        return Data

    
    def get_gyro_degree(self,gyro,dt):
        self.roll += (self.pregx + gyro[0])*dt / 2
        self.pregx = gyro[0]


        self.pitch += (self.pregy + gyro[1]) * dt / 2
        self.pregy = gyro[1]

        self.yaw += (self.pregz + gyro[2]) * dt / 2
        self.pregz = gyro[2]

        self.roll  += self.pitch * math.sin(gyro[2] * dt * math.pi/180)    
        self.pitch -= self.roll * math.sin(gyro[2] * dt* math.pi/180) 

        u = np.array([ 
            [self.roll],
            [self.pitch],
            [self.yaw]
        ]) 
        return u


    def get_acc_degree(self,acc):

        ax = acc[0]
        ay = acc[1]
        az = acc[2]
         
        acc_pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az))) #* math.pi/180

        # roll        
        acc_roll  = -math.degrees(math.atan2(ay, az))# * math.pi/180
        if acc_roll >= 0:
            acc_roll = 180 - acc_roll
        if acc_roll < 0 :
            acc_roll = -180 - acc_roll

        return acc_roll,acc_pitch
    
    def convert_euler_to_Rxyz(self,x):
        c1 = np.cos(x[0][0])
        s1 = np.sin(x[0][0])
        c2 = np.cos(x[1][0])
        s2 = np.sin(x[1][0])
        c3 = np.cos(x[2][0])
        s3 = np.sin(x[2][0])
        Rx = np.array([
            [1, 0, 0],
            [0, c1, -s1],
            [0, s1, c1],
        ])
        Ry = np.array([
            [c2, 0, s2],
            [0, 1, 0],
            [-s2, 0, c2],
        ])
        Rz = np.array([
            [c3, -s3, 0],
            [s3, c3, 0],
            [0, 0, 1],
        ])
        Rxyz = Rz @ Ry @ Rx
        return Rxyz
    
    def GetSensorData(self ,recv_data):

        time_stamp = time.time() - self.ut
        dt=time_stamp - self.pre_time_stamp
        self.pre_time_stamp=time_stamp

        x = np.array([[0], [0], [0]])
        P = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])
    

        # get raw data
        self.acc[0] =self.BinaryCalc(recv_data[8],recv_data[9])
        self.acc[1] =self.BinaryCalc(recv_data[10],recv_data[11])
        self.acc[2] =self.BinaryCalc(recv_data[12],recv_data[13])
        self.gyro[0] = self.BinaryCalc(recv_data[16],recv_data[17])
        self.gyro[1] = self.BinaryCalc(recv_data[18],recv_data[19])
        self.gyro[2] = self.BinaryCalc(recv_data[20],recv_data[21])
        # self.acc = -np.array([self.BinaryCalc(recv_data[8],recv_data[9]),
        #                  self.BinaryCalc(recv_data[10],recv_data[11]),
        #                  self.BinaryCalc(recv_data[12],recv_data[13])])

        # self.gyro = np.array([self.BinaryCalc(recv_data[16],recv_data[17]), 
        #                  self.BinaryCalc(recv_data[18],recv_data[19]),
        #                  self.BinaryCalc(recv_data[20],recv_data[21])])

        self.acc[0] = self.acc[0]/2048
        self.acc[1] = self.acc[1]/2048
        self.acc[2] = self.acc[2]/2048

        self.gyro[0] = self.gyro[0]/16.4
        self.gyro[1] = self.gyro[1]/16.4
        self.gyro[2] = self.gyro[2]/16.4

        u = self.get_gyro_degree(self.gyro,dt)
        z = self.get_acc_degree(self.acc)
        R = np.diag([1.0*dt**2, 1.0*dt**2])
        Q = np.diag([1.74E-2*dt**2, 1.74E-2*dt**2, 1.74E-2*dt**2])

        x, P = ekf(x, u, z, P, R, Q)
        Rxyz = self.convert_euler_to_Rxyz(x)


        # print("Time stamp:",time_stamp)
        # print("dt:",dt)
        # print("data type :",type(recv_data))
        # print("recv raw data:",recv_data )
        # print("------------------------------------")
        # print("X acc is :",self.acc[0] )
        # print("Y acc is :",self.acc[1] )
        # print("Z acc is :",self.acc[2] )
        # print("X gyro is :",self.gyro[0] )
        # print("Y gyro is :",self.gyro[1] )
        # print("Z gyro is :",self.gyro[2] )
        # print("------------------------------------")
        # print("gyro deg:",u)
        # print("acc deg:",z) 
        print("filtering degree(euler)\n", x)
        # print("filtering degree\n", Rxyz)
        # print(" ")
    
        return time_stamp,x

if __name__ == "__main__":

    acc = None
    ts_pre = None

    imu = IMU()
    ser = serial.Serial(
        # port = "/dev/ttyACM0",  #Linux
        port = 'COM3',            #Windows
        baudrate = 115200,
        #parity = serial.PARITY_NONE,
        bytesize = serial.EIGHTBITS,
        stopbits = serial.STOPBITS_ONE,
        )

    while(True) :
        if ser.in_waiting > 0:
            print('in_waiting is',ser.in_waiting)
            recv_data = ser.read(28)
            imu.GetSensorData(recv_data)

        
            