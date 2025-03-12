import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../gyroscope_output.txt', delimiter =', ')
plt.figure(figsize=(10,6))
plt.plot(df['t'], df['w'], label = 'angular velocity')
plt.plot(df['t'], df['w_g'], label = 'measured angular velocity')
plt.title('Angular Velocity v. Time (GYROSCOPE)')
plt.xlabel('time (s)')
plt.ylabel('angular velocity (rad/s)')
#plt.legend()
plt.grid(True)
plt.savefig('gyroscope(WvT).png')