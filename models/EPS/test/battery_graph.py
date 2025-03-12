import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../battery_output.txt', delimiter =', ')
plt.figure(figsize=(10,6))
plt.plot(df['t'], df['V_t'])
#plt.plot(df['V'], df['I2'])
plt.title('Output Voltage Of Battery')
plt.xlabel('time (s)')
plt.ylabel('Terminal Voltage (V)')
#plt.legend()
plt.grid(True)
plt.savefig('OV_Battery.png')

plt.figure(figsize=(10,6))
plt.plot(df['t'], df['SOC'])
plt.title('Sate Of Charge Of Battery')
plt.xlabel('time (s)')
plt.ylabel('State Of Charge (SOC)')
#plt.legend()
plt.grid(True)
plt.savefig('SOC_Battery.png')