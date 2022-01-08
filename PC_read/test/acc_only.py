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

file = open("accel_data.csv", "w")
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

            Y_acc =BinaryCalc(recv_data[10],recv_data[11])
            Y_acc = Y_acc/2048

            Z_acc =BinaryCalc(recv_data[12],recv_data[13])
            Z_acc = Z_acc/2048

            print("recv raw data:",recv_data )
            print("X acc is :",X_acc )
            print("X gyro is :",Y_acc )
            print("Z acc is :",Z_acc )
            print("")

            
            print("deg = ",deg)
            print("")
            file.write(str(time_stamp) + "," + str(X_acc) + "," + str(Y_acc) + "," + str(Z_acc) + "\n")  

except KeyboardInterrupt:
    file.close()
    
    df = pd.read_csv('accel_data.csv', names=['num1', 'num2','num3','num4'])
    # plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
    plt.plot(range(0,count),df['num2'])
    plt.plot(range(0,count),df['num3'])
    plt.plot(range(0,count),df['num4'])
    plt.show()