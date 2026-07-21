import pandas as pd
import glob
import os
import numpy as np

files = glob.glob('/home/kaikeller/Purdue/AOL/MAV/SD-Logging/720-logs/*.CSV')
files.sort()

for f in files:
    try:
        df = pd.read_csv(f)
        if 'gx' in df.columns and 'gy' in df.columns and 'gz' in df.columns:
            max_gx = df['gx'].abs().max()
            max_gy = df['gy'].abs().max()
            max_gz = df['gz'].abs().max()
            
            # altitude diff: prefer gps_alt_m if it exists and has non-null values
            alt_diff = 0
            if 'gps_alt_m' in df.columns and not df['gps_alt_m'].isnull().all():
                alt_diff = df['gps_alt_m'].max() - df['gps_alt_m'].min()
            elif 'pu' in df.columns:
                alt_diff = df['pu'].max() - df['pu'].min()
                
            time_s = (df['ms'].max() - df['ms'].min()) / 1000.0 if 'ms' in df.columns else 0
            
            print(f"{os.path.basename(f)}: time {time_s:.1f}s, alt_diff {alt_diff:.2f}m, max_gx {max_gx:.2f}, max_gy {max_gy:.2f}, max_gz {max_gz:.2f}")
    except Exception as e:
        print(f"Error reading {f}: {e}")

