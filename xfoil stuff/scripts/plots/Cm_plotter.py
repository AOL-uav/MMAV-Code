import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import plotly.express as px
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PLOT_DIR = PROJECT_ROOT / "results" / "runs" / "plots"

def extract_camber(airfoil_name):
    match = re.search(r'(\d)(\d)(\d{2})$', str(airfoil_name))
    if match:
        return float(match.group(1))
    return 0.0

def generate_plot_set(df, y_col, y_label, title_prefix, file_prefix, target_re, highlights, cmap, norm, airfoil_colors):
    airfoils = df['Airfoil'].unique()

    # 1. Interactive Plotly Output
    print(f"Generating interactive HTML plot for {y_col}...")
    fig = px.line(df, x='alpha', y=y_col, color='Airfoil',
                  color_discrete_map=airfoil_colors,
                  hover_name='Airfoil',
                  title=f'{title_prefix} vs Angle of Attack (Re = {target_re})')
    
    fig.add_hline(y=0, line_dash="dash", line_color="black")
    fig.write_html(f"{file_prefix}_interactive_Re{target_re}.html")
    
    # 2. Static Heatmap Output
    print(f"Generating static heatmap plot for {y_col}...")
    plt.figure(figsize=(12, 6), dpi=300)
    for airfoil in airfoils:
        airfoil_data = df[df['Airfoil'] == airfoil]
        c_val = airfoil_data['Camber'].iloc[0]
        plt.plot(airfoil_data['alpha'], airfoil_data[y_col], color=cmap(norm(c_val)), linewidth=1.5, alpha=0.7)
        
    plt.axhline(0, color='black', linestyle='--', linewidth=1.5)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=plt.gca())
    cbar.set_label('Max Camber (%)', fontsize=10)
    
    plt.title(f'{title_prefix} by Camber vs Alpha (Re = {target_re})', fontsize=14)
    plt.xlabel('Angle of Attack $\\alpha$ [deg]', fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    plt.savefig(f"{file_prefix}_heatmap_Re{target_re}.png")
    plt.close()

    # 3. Selective Highlight Output
    print(f"Generating selective highlight plot for {y_col}...")
    plt.figure(figsize=(12, 6), dpi=300)
    for airfoil in airfoils:
        airfoil_data = df[df['Airfoil'] == airfoil]
        c_val = airfoil_data['Camber'].iloc[0]
        if airfoil in highlights:
            plt.plot(airfoil_data['alpha'], airfoil_data[y_col], color=cmap(norm(c_val)), linewidth=2.5, label=f"{airfoil} ({c_val}% Camber)", zorder=10)
        else:
            plt.plot(airfoil_data['alpha'], airfoil_data[y_col], color='grey', linewidth=1.0, alpha=0.15, zorder=1)

    plt.axhline(0, color='black', linestyle='--', linewidth=1.5)
    plt.title(f'{title_prefix} Selected Highlights (Re = {target_re})', fontsize=14)
    plt.xlabel('Angle of Attack $\\alpha$ [deg]', fontsize=12)
    plt.ylabel(y_label, fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    if any(h in airfoils for h in highlights):
        plt.legend()
    plt.tight_layout()
    plt.savefig(f"{file_prefix}_selective_Re{target_re}.png")
    plt.close()

def plot_aerodynamics(filename, target_re, highlights, c_m, b_m, rho, nu):
    print(f"Loading data from {filename}...")
    df = pd.read_csv(PROJECT_ROOT / "data" / "polars" / filename, sep=r'\s+')
    df_filtered = df[df['Re'] == target_re].copy()
    
    if df_filtered.empty:
        print(f"Warning: No data found for Re = {target_re}.")
        return

    # Physics calculations for absolute torque (N*m)
    S_m2 = c_m * b_m
    V_mps = (target_re * nu) / c_m
    q = 0.5 * rho * (V_mps ** 2)
    
    df_filtered['Camber'] = df_filtered['Airfoil'].apply(extract_camber)
    df_filtered['M_Nm'] = df_filtered['CM'] * q * S_m2 * c_m
    df_sorted = df_filtered.sort_values(by=['Camber', 'Airfoil', 'alpha'])

    # Display calculated physical params
    print(f"\n--- Physical Parameters Used ---")
    print(f"Chord (c): {c_m} m, Span (b): {b_m} m, Area (S): {S_m2} m^2")
    print(f"Calculated Velocity for Re={target_re}: {V_mps:.2f} m/s")
    print(f"Dynamic Pressure (q): {q:.2f} Pa\n")

    # Setup colors
    cmap = plt.get_cmap('coolwarm')
    norm = mcolors.Normalize(vmin=df_sorted['Camber'].min(), vmax=df_sorted['Camber'].max())
    airfoil_colors = {af: mcolors.to_hex(cmap(norm(df_sorted[df_sorted['Airfoil'] == af]['Camber'].iloc[0]))) for af in df_sorted['Airfoil'].unique()}

    # Generate Cm plots
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    generate_plot_set(df_sorted, 'CM', 'Pitching Moment Coefficient ($C_m$)', 
                      'Pitching Moment ($C_m$)', str(PLOT_DIR / 'Cm_comparison'), 
                      target_re, highlights, cmap, norm, airfoil_colors)
                      
    # Generate Absolute Moment plots
    generate_plot_set(df_sorted, 'M_Nm', 'Absolute Pitching Moment $M$ (N·m)', 
                      'Absolute Pitching Torque ($M$)', str(PLOT_DIR / 'M_abs_comparison'), 
                      target_re, highlights, cmap, norm, airfoil_colors)

    print("Done! Created all HTML and PNG files.")

if __name__ == "__main__":
    my_highlights = ['NACA_0004', 'NACA_0012', 'NACA_1412', 'NACA_2412', 'NACA_3412', 'NACA_4412', 'NACA_5412', 'NACA_6412', 'NACA_7412', 'NACA_8412'] 

    #nobs    
    CHORD_METERS = 0.1       # c
    SPAN_METERS = 1.0        # b
    AIR_DENSITY = 1.225      # rho (kg/m^3 at sea level)
    KIN_VISCOSITY = 1.48e-5  # nu (m^2/s at standard temp)
    
    plot_aerodynamics('Currentmegapolar.dat', target_re=75000, highlights=my_highlights, 
                      c_m=CHORD_METERS, b_m=SPAN_METERS, rho=AIR_DENSITY, nu=KIN_VISCOSITY)
