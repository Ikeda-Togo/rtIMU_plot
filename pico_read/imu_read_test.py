from machine import Pin, UART
import time
import math

class IMU:
    def __init__(self):#constructor

        self.count = 0
        self.ut = time.time()
        self.pre_time_stamp = 0

        self.acc=[0] * 3

        self.gyro_deg = [0] * 3
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

        self.roll  += self.pitch * math.sin(gyro[2] * dt * math.pi/180) ;   
        self.pitch -= self.roll * math.sin(gyro[2] * dt* math.pi/180) ; 
        
        return self.roll , self.pitch , self.yaw


    def get_acc_degree(self,acc):

        ax = acc[0]
        ay = acc[1]
        az = acc[2]
         
        acc_pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az))) #* math.pi/180

        # pitch
        # if  az >= 0:       
        #     acc_pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))# * math.pi/180
        # elif az < 0:
        #     acc_pitch = math.degrees(math.atan2(ax, -math.sqrt(ay * ay + az * az)))# * math.pi/180
        # if acc_pitch >=0:
        #     acc_pitch = 180 - acc_pitch
        # elif acc_pitch < 0:
        #     acc_pitch = -180 - acc_pitch


        # roll        
        acc_roll  = -math.degrees(math.atan2(ay, az))# * math.pi/180
        if acc_roll >= 0:
            acc_roll = 180 - acc_roll
        if acc_roll < 0 :
            acc_roll = -180 - acc_roll

        return acc_roll,acc_pitch
    
    def GetSensorData(self ,recv_data):

        time_stamp = time.time() - self.ut
        dt=time_stamp - self.pre_time_stamp
        self.pre_time_stamp=time_stamp
    

        # get raw data
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


        self.gyro_deg = self.get_gyro_degree(self.gyro,dt)
        acc_roll,acc_pitch = self.get_acc_degree(self.acc)

        # filter degree
        filter_roll = 0.995 * (self.pre_filter_roll + self.gyro[0] * dt) + 0.005 * acc_roll
        self.pre_filter_roll=filter_roll
        filter_pitch = 0.995 * (self.pre_filter_pitch + self.gyro[1] * dt) + 0.005 * acc_pitch
        self.pre_filter_pitch=filter_pitch

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
        print("gyro deg:",self.gyro_deg)
        print("acc deg:",acc_roll,acc_pitch) 
        print("filtering roll",filter_roll)
        print("filtering pitch",filter_pitch)
        print(" ")
    
        return time_stamp,acc_pitch,self.gyro_deg[1],filter_pitch
    
if __name__ == "__main__":
    imu = IMU()
    counter=0
    uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
    u= UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
    txData = b'\xff\xffRT9A\x12\x1c\xe5\xfeF\x01\xe7\xf7g\n\x01\x00\x02\x00\xff\xff\xb2\x00:\x01\xd5\xff'
    rxData = bytes()
    
    print('UART test')
    uart0.write(txData)
    time.sleep(0.1)
    if u.any() > 0:
        print('u.any is ',u.any())
        recv_data = u.read(28)
        print('recv-data-is',recv_data)
        imu.GetSensorData(recv_data)