import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# PHYSICAL PROTOTYPE PARAMETERS
# ==========================================
WING_SPAN_M = 0.9      
WING_CHORD_M = 0.15    
S_WING = WING_SPAN_M * WING_CHORD_M

FLAP_AREA_MM2 = 6877.0
S_FLAP = FLAP_AREA_MM2 / 1_000_000  # Convert to m^2

# Lift slope of a low-aspect-ratio flappy surface (approx Cl per degree)
FLAP_CL_ALPHA = 0.05 

# ==========================================

def plot_weathervane_equilibrium():
    # Sweep lever arm from 1mm (0.1cm) to 10cm
    lever_arms_cm = np.linspace(0.1, 10, 200)
    lever_arms_m = lever_arms_cm / 100.0

    # Airfoil Test Cases (Approximate Cm values)
    airfoils = {
        "NACA 0012 (Symmetrical)": {"Cm": 0.0, "color": "green"},
        "NACA 2412 (Mild Camber)": {"Cm": -0.05, "color": "orange"},
        "NACA 4412 (High Camber)": {"Cm": -0.10, "color": "red"}
    }

    plt.figure(figsize=(8, 6), dpi=150)

    for name, data in airfoils.items():
        Cm = data["Cm"]
        
        # Calculate trim angle: alpha = (S_wing * c * Cm) / (S_flap * lever * Cl_alpha)
        # We use absolute value for Cm to show "Nose Down Angle" as a positive drop on the graph
        trim_angles_down = abs((S_WING * WING_CHORD_M * Cm) / (S_FLAP * lever_arms_m * FLAP_CL_ALPHA))
        
        # Cap the angle at 90 degrees (a straight vertical dive)
        trim_angles_down = np.clip(trim_angles_down, 0, 90)
        
        plt.plot(lever_arms_cm, trim_angles_down, label=f"{name} ($C_m$ = {Cm})", 
                 color=data["color"], linewidth=3)

    # Graph Formatting
    plt.title(f"Weather Vane Equilibrium Angle vs. Lever Arm\n(Assuming {FLAP_AREA_MM2} mm² Flappy Area)", fontsize=14, fontweight='bold')
    plt.xlabel("Flappy Lever Arm Distance from CG (cm)", fontsize=12)
    plt.ylabel("Nose-Down Pitch Angle Required to Trim (Degrees)", fontsize=12)
    
    # Highlight the "almost zero" zone
    plt.axvspan(0, 2, color='grey', alpha=0.2, label='The "Almost Zero" Danger Zone')
    
    plt.xlim(0, 10)
    plt.ylim(0, 95)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc="upper right", fontsize=10)
    
    # Add a horizontal line showing a "reasonable" 5-degree nose-down flight
    plt.axhline(5, color='black', linestyle='--', linewidth=1.5)
    plt.text(10.2, 5, 'Flyable Limit (-5°)', va='center')

    plt.tight_layout()
    plt.savefig("wingAnalysis/weathervane_equilibrium.png")
    plt.show()

if __name__ == "__main__":
    plot_weathervane_equilibrium()