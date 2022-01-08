import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('accel_data.csv', names=['num1', 'num2','num3'])
# plt.plot(range(0,count),df['num1'],marker="o",markersize=2)
plt.plot(df['num1'],df['num2'])
plt.show()