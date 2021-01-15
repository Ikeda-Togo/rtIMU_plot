import serial
import struct
import time
import pandas as pd
import matplotlib.pyplot as plt

ser = serial.Serial(
      port = "/dev/ttyACM0",
      baudrate = 115200,
      #parity = serial.PARITY_NONE,
      bytesize = serial.EIGHTBITS,
      stopbits = serial.STOPBITS_ONE,
      #timeout = 0.1,
      #xonxoff = 0,
      #rtscts = 0,
      )

file = open("sensor_data.csv", "w")
deg = 0
count = 0
ut = time.time()

#バイナリデータの上位と下位を計算
def BinaryCalc (low_bit, high_bit):
    Low_data=low_bit
    High_data=high_bit << 8
    Data =Low_data + High_data

    if Data >= 32767:
        Data = Data -65535

    return Data


try:
    while True:
        if ser.in_waiting > 0:

            count =count+1
            time_stamp = time.time() - ut
            print("Time stamp",time_stamp)
            
            recv_data = ser.read(28)
            print("data type :",type(recv_data))


            X_acc =BinaryCalc(recv_data[8],recv_data[9])
            X_acc = X_acc/2048

            Z_acc =BinaryCalc(recv_data[12],recv_data[13])
            Z_acc = Z_acc/2048

            X_gyro = BinaryCalc(recv_data[16],recv_data[17])
            X_gyro = X_gyro/16.4

            print("recv raw data:",recv_data )
            print("X acc is :",X_acc )
            print("X gyro is :",X_gyro )
            print("Z acc is :",Z_acc )
            print("")

            if X_acc > 1:
                X_acc = 1
            if Z_acc > 2:
                Z_acc = 2


            if 0 <= X_acc <= 1 and 0 <= Z_acc <= 1:
                deg = 90 * X_acc

            if 0 <= X_acc <= 1 and 1 < Z_acc <= 2:
                deg = 180 - 90 * X_acc
            
            if -1 <= X_acc < 0 and 0 <= Z_acc <= 1:
                deg = 90 * X_acc

            if -1 <= X_acc < 0 and 1 < Z_acc <= 2:
                deg = -180 - 90 * X_acc
            
            else :
                print("OUT RANGE")

            print("deg = ",deg)
            print("")
            file.write(str(time_stamp) + "," + str(deg) + "," + str(X_gyro) + "\n")  

except KeyboardInterrupt:
    file.close()
    
    df = pd.read_csv('sensor_data.csv', names=['num1', 'num2','num3'])
    # plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
    plt.plot(range(0,count),df['num2'])
    plt.plot(range(0,count),df['num3'])
    plt.show()