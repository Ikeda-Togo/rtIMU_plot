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

try:
    while True:
        if ser.in_waiting > 0:

            count =count+1
            time_stamp = time.time() - ut
            print("Time stamp",time_stamp)
            
            recv_data = ser.read(28)
            print(type(recv_data))


            x_acc_L=recv_data[8]
            x_acc_H=recv_data[9] << 8

            X_acc =x_acc_L+x_acc_H
            if X_acc >= 32767:
                X_acc = X_acc -65535

            X_acc = X_acc/2048


            z_acc_L=recv_data[12]
            z_acc_H=recv_data[13] << 8

            Z_acc =z_acc_L+z_acc_H
            if Z_acc >= 32767:
                Z_acc = Z_acc -65535

            Z_acc = Z_acc/2048

            #a = struct.unpack_from("B",recv_data ,16)
            #b = struct.unpack_from("B",recv_data ,17)

            print("recv raw data:",recv_data )
            print("X acc is :",X_acc )
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
                pass
            
            print("deg = ",deg)
            print("")
            file.write(str(count) + "," + str(deg) + "," + str(Z_acc) + "\n")  

except KeyboardInterrupt:
    file.close()
    
    df = pd.read_csv('accel_data.csv', names=['num1', 'num2','num3'])
    # plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
    plt.plot(range(0,count),df['num2'])
    plt.show()