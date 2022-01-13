import serial
import time

ser = serial.Serial(
    # port = "/dev/ttyACM0",  #Linux
    port = 'COM3',            #Windows
    baudrate = 115200,
    #parity = serial.PARITY_NONE,
    bytesize = serial.EIGHTBITS,
    stopbits = serial.STOPBITS_ONE,
    )
# txData = b'\xff\xffRT9A\x12\x1c\xe5\xfeF\x01\xe7\xf7g\n\x01\x00\x02\x00\xff\xff\xb2\x00:\x01\xd5\xff'
# ser.write(txData)
time.sleep(1)
while ser.in_waiting > 0:
    print('in_waiting is',ser.in_waiting)
    recv_data = ser.read(ser.in_waiting)
    print(recv_data)