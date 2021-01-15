import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('down_step.csv', names=['num1', 'num2','num3','num4'])

plt.figure()
plt.plot(df['num1'],df['num2'],label='acc_angle')
plt.plot(df['num1'],df['num3'],label= "gyro_angle")
plt.plot(df['num1'],df['num4'],label= "filter_angle")
plt.legend()
plt.show()