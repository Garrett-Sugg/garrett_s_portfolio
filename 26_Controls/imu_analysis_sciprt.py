# IMU data analysis script
# TODO

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv(r'D:\imu_data.txt') # file path must match
df.head()

df_2 = df[df['id'] == 2] # filter by sender ID 

window = 20

df_2['ax_smoothed'] = df_2['ax'].rolling(window, min_periods=1).mean()
df_2['ax_smoothed'] = df_2['ay'].rolling(window, min_periods=1).mean()
df_2['ax_smoothed'] = df_2['az'].rolling(window, min_periods=1).mean()

plt.figure(figsize=(12, 6))

plt.plot(df_2['t(ms)'], df_2['ax_smoothed'], label='ax (smooth)')
plt.plot(df_2['t(ms)'], df_2['ax_smoothed'], label='ay (smooth)')
plt.plot(df_2['t(ms)'], df_2['ax_smoothed'], label='az (smooth)')

plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Smoothed Acceleration Components')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# compute standard Euclidean norm of acceleration
'''
df_2['acceleration_magnitude_vector'] = np.sqrt(
    df_2['ax']**2 +
    df_2['ay']**2 +
    df_2['az']**2
)
'''

df_2['acceleration_magnitude_vector_smoothed'] = np.sqrt(
    df_2['ax_smoothed']**2 +
    df_2['ax_smoothed']**2 +
    df_2['ax_smoothed']**2
)

plt.figure(figsize=(12, 6))

plt.plot(df_2['t(ms)'], df_2['acceleration_magnitude_vector_smoothed'], label='Smoothed Magnitude', linewidth=2)

plt.xlabel('Time (ms)')
plt.ylabel('Acceleration Magnitude (m/s²)')
plt.title('Smoothed Acceleration Magnitude')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

'''
plt.figure(figsize=(12, 6))

plt.scatter(df_2['t(ms)'], df_2['acceleration_magnitude_vector'], s=10)

plt.xlabel('Time (ms)')
plt.ylabel('Acceleration Magnitude (m/s^2)')
plt.title('Magnitude of Acceleration vs Time')

plt.grid(True)
plt.tight_layout()
plt.show()

plt.plot(df_2['t(ms)'], df_2['ax'], label='ax')
plt.plot(df_2['t(ms)'], df_2['ay'], label='ay')
plt.plot(df_2['t(ms)'], df_2['az'], label='az')

plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Acceleration vs Time - Module 2')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
'''