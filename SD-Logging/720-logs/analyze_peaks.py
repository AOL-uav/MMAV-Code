import pandas as pd
import numpy as np

files = [
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_001.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_002-card2.CSV',
    '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/0720_005.CSV'
]

for f in files:
    df = pd.read_csv(f)
    t = df['ms'] / 1000.0
    rot_mag = np.sqrt(df['gx']**2 + df['gy']**2 + df['gz']**2)
    max_rot_idx = rot_mag.idxmax()
    t_max_rot = t.iloc[max_rot_idx]
    
    if 'gps_alt_m' in df.columns and not df['gps_alt_m'].isnull().all():
        alt = df['gps_alt_m']
    else:
        alt = df['pu']
        
    print(f"{f.split('/')[-1]}: t_max_rot={t_max_rot:.1f}s, max_rot={rot_mag.iloc[max_rot_idx]:.2f}, total_t={t.iloc[-1]:.1f}s")
