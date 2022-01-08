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

file = open("gyro_data.csv", "w")
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


            X_gyro =BinaryCalc(recv_data[16],recv_data[17])
            X_gyro = X_gyro/16.4

            Y_gyro =BinaryCalc(recv_data[18],recv_data[19])
            Y_gyro = Y_gyro/16.4

            Z_gyro =BinaryCalc(recv_data[20],recv_data[21])
            Z_gyro = Z_gyro/16.4

            print("recv raw data:",recv_data )
            print("X gyro is :",X_gyro )
            print("Y gyro is :",Y_gyro )
            print("Z gyro is :",Z_gyro )
            print("")

            
            print("deg = ",deg)
            print("")
            file.write(str(time_stamp) + "," + str(X_gyro) + "," + str(Y_gyro) + "," + str(Z_gyro) + "\n")  

except KeyboardInterrupt:
    file.close()
    
    df = pd.read_csv('gyro_data.csv', names=['num1', 'num2','num3','num4'])
    # plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
    plt.plot(range(0,count),df['num2'])
    plt.plot(range(0,count),df['num3'])
    plt.plot(range(0,count),df['num4'])
    plt.show()