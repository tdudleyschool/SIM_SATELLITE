import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../solar_output.txt', delimiter =', ')
print(df)
plt.figure(figsize=(10,6))
plt.plot(df['V'], df['I'])
#plt.plot(df['V'], df['I2'])
plt.title('IV Curve For Solar Pannel')
plt.xlabel('V')
plt.ylabel('I')
#plt.legend()
plt.grid(True)
plt.savefig('plot.png')