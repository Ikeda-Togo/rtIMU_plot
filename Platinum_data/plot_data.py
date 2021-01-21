import pandas as pd
import matplotlib.pyplot as plt

# フォントサイズ
plt.rcParams["font.size"] = 30

# file_name = 'down_step'
file_name = 'step_up'
df = pd.read_csv(file_name+'.csv', names=['num1', 'num2','num3','num4'])

fig=plt.figure()
plt.plot(df['num1'],df['num2'],label='acc_pitch_angle')
# plt.plot(df['num1'],df['num3'],label= "gyro_angle")
plt.plot(df['num1'],df['num4'],label= "pitch_angle", linewidth = 3)
plt.legend()
fig.text(0.88, 0.06, '[s]')
fig.text(0.05, 0.87, '[deg]')

# Show the major grid lines with dark grey lines
plt.grid(b=True, which='major', color='#666666', linestyle='-')

# Show the minor grid lines with very faint and almost transparent grey lines
plt.minorticks_on()
plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)



manager = plt.get_current_fig_manager()
manager.window.showMaximized()

plt.show()
fig.savefig(file_name+'_graph.png')
