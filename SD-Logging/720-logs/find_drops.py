import pandas as pd
import glob
import os

files = glob.glob('/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/*.CSV')
print(f"Found {len(files)} CSV files.")

for f in files:
    try:
        df = pd.read_csv(f)
        if 'pu' in df.columns:
            alt_diff = df['pu'].max() - df['pu'].min()
            time_s = (df['ms'].max() - df['ms'].min()) / 1000.0 if 'ms' in df.columns else 0
            
            # looking for drops: significant altitude change
            print(f"{os.path.basename(f)}: duration {time_s:.1f}s, altitude change {alt_diff:.2f}m")
    except Exception as e:
        print(f"Error reading {f}: {e}")

