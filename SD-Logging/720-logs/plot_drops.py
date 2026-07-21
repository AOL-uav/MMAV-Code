import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Files that look like rotational drops based on max rotation rates
files = [
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_002-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_005.CSV'
]

fig, axs = plt.subplots(3, 1, figsize=(10, 15))

for f in files:
    df = pd.read_csv(f)
    name = f.split('/')[-1]
    
    # Time in seconds
    t = df['ms'] / 1000.0
    t = t - t.iloc[0]
    
    # Altitude - use gps_alt_m if available and not all NaN, else use pu
    if 'gps_alt_m' in df.columns and not df['gps_alt_m'].isnull().all():
        alt = df['gps_alt_m']
        # center altitude to start at 0 for comparison
        alt = alt - alt.dropna().iloc[0]
    else:
        alt = df['pu']
        alt = alt - alt.iloc[0]
        
    # Horizontal velocity magnitude
    h_vel = np.sqrt(df['vn']**2 + df['ve']**2)
    
    # Rotation rate magnitude
    rot_mag = np.sqrt(df['gx']**2 + df['gy']**2 + df['gz']**2)
    
    axs[0].plot(t, alt, label=name)
    axs[1].plot(t, h_vel, label=name)
    axs[2].plot(t, rot_mag, label=name)

axs[0].set_title('Altitude vs Time')
axs[0].set_ylabel('Altitude (m)')
axs[0].legend()
axs[0].grid(True)

axs[1].set_title('Horizontal Velocity vs Time')
axs[1].set_ylabel('Velocity (m/s)')
axs[1].legend()
axs[1].grid(True)

axs[2].set_title('Rotation Rate Magnitude vs Time')
axs[2].set_ylabel('Rotation Rate (rad/s)')
axs[2].legend()
axs[2].grid(True)

plt.xlabel('Time (s)')
plt.tight_layout()
plt.savefig('/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/rotational_drops_overview.png')
print("Plot saved to rotational_drops_overview.png")

