import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

files = [
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_002-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_005.CSV'
]
labels = [
    'Flight 001 (Card 2)',
    'Flight 001',
    'Flight 002 (Card 2)',
    'Flight 005'
]

# Set up the figure
fig = plt.figure(figsize=(16, 12))
fig.suptitle('Rotational Drops Analysis', fontsize=20, fontweight='bold')

# 1. 3D Trajectory Plot
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.set_title('3D Trajectory')
ax1.set_xlabel('East (m)')
ax1.set_ylabel('North (m)')
ax1.set_zlabel('Relative Altitude (m)')

# 2. Altitude vs Aligned Time
ax2 = fig.add_subplot(2, 2, 2)
ax2.set_title('Altitude Profile (Aligned at Drop Start)')
ax2.set_xlabel('Time from drop (s)')
ax2.set_ylabel('Relative Altitude (m)')
ax2.grid(True, linestyle='--', alpha=0.7)

# 3. Horizontal Trajectory (Top-Down)
ax3 = fig.add_subplot(2, 2, 3)
ax3.set_title('Horizontal Trajectory (Top-Down)')
ax3.set_xlabel('East (m)')
ax3.set_ylabel('North (m)')
ax3.grid(True, linestyle='--', alpha=0.7)
ax3.axis('equal')

# 4. Rotation Rate Profile
ax4 = fig.add_subplot(2, 2, 4)
ax4.set_title('Rotation Rate (Aligned at Drop Start)')
ax4.set_xlabel('Time from drop (s)')
ax4.set_ylabel('Rotation Magnitude (rad/s)')
ax4.grid(True, linestyle='--', alpha=0.7)

for f, label in zip(files, labels):
    df = pd.read_csv(f)
    t = df['ms'] / 1000.0
    
    # Calculate magnitudes
    rot_mag = np.sqrt(df['gx']**2 + df['gy']**2 + df['gz']**2)
    
    # Find drop point based on max rotation
    max_rot_idx = rot_mag.idxmax()
    t_drop = t.iloc[max_rot_idx]
    
    # Align time
    t_aligned = t - t_drop
    
    # Altitude logic
    if 'gps_alt_m' in df.columns and not df['gps_alt_m'].isnull().all():
        alt = df['gps_alt_m']
    else:
        alt = df['pu']
        
    # We want to smooth altitude slightly or just take it as is
    alt = alt - alt.min() # set bottom to 0 for consistency
    
    # Positions (integrate velocity if pn/pe are drifting, but we'll use pn/pe)
    # Re-center position at the drop point
    pn = df['pn'] - df['pn'].iloc[max_rot_idx]
    pe = df['pe'] - df['pe'].iloc[max_rot_idx]
    
    # Filter to interesting region (e.g. -5 to +20 seconds around drop)
    mask = (t_aligned > -10) & (t_aligned < 30)
    
    # Plotting
    ax1.plot(pe[mask], pn[mask], alt[mask], label=label, linewidth=2)
    ax2.plot(t_aligned[mask], alt[mask], label=label, linewidth=2)
    ax3.plot(pe[mask], pn[mask], label=label, linewidth=2)
    ax4.plot(t_aligned[mask], rot_mag[mask], label=label, linewidth=2)

# Legends
ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
out_path = '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/drop_analysis.png'
plt.savefig(out_path, dpi=300, bbox_inches='tight')
print(f"Saved {out_path}")
