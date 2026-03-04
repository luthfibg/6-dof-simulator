import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import Environment  # Modul Python yang memodelkan Environment.hpp (getDensity, dll.)

# Asumsi awal: kita ingin membandingkan dua model
# - trajectory_old.csv (model konstan)
# - trajectory_new.csv (model environment)
# Namun di versi sekarang, simulasi hanya menulis trajectory.csv.
# Jadi kita buat fallback: jika hanya trajectory.csv yang ada,
# kita plot satu model saja (Environment) dan tetap bandingkan
# model atmosfer (eksponensial vs ISA Environment) di subplot ke-4.

mode = None

if os.path.exists('trajectory_old.csv') and os.path.exists('trajectory_new.csv'):
    mode = 'dual'
    df_old = pd.read_csv('trajectory_old.csv')
    df_new = pd.read_csv('trajectory_new.csv')
elif os.path.exists('trajectory.csv'):
    mode = 'single'
    df_new = pd.read_csv('trajectory.csv')
else:
    print("Tidak menemukan trajectory_old.csv & trajectory_new.csv ataupun trajectory.csv.\n"
          "Jalankan simulasi dulu (missile_sim.exe) untuk menghasilkan data.")
    raise SystemExit(0)

try:
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. Altitude vs Time
    if mode == 'dual':
        axes[0,0].plot(df_old['Time(s)'], df_old['Altitude(m)'], 'b--', label='Model Konstan', alpha=0.7)
        axes[0,0].plot(df_new['Time(s)'], df_new['Altitude(m)'], 'r-', label='Model Environment', linewidth=2)
    else:  # single
        axes[0,0].plot(df_new['Time(s)'], df_new['Altitude(m)'], 'r-', label='Model Environment', linewidth=2)
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Altitude (m)')
    axes[0,0].set_title('Altitude (Environment Model)')
    axes[0,0].legend()
    axes[0,0].grid(True, alpha=0.3)
    
    # 2. Speed vs Time
    if mode == 'dual':
        speed_old = np.sqrt(df_old['Vx(m/s)']**2 + df_old['Vy(m/s)']**2 + df_old['Vz(m/s)']**2)
        axes[0,1].plot(df_old['Time(s)'], speed_old, 'b--', label='Model Konstan', alpha=0.7)
    speed_new = np.sqrt(df_new['Vx(m/s)']**2 + df_new['Vy(m/s)']**2 + df_new['Vz(m/s)']**2)
    axes[0,1].plot(df_new['Time(s)'], speed_new, 'r-', label='Model Environment', linewidth=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Speed (m/s)')
    axes[0,1].set_title('Speed Comparison')
    axes[0,1].legend()
    axes[0,1].grid(True, alpha=0.3)
    
    # 3. Trajectory (X vs Y)
    if mode == 'dual':
        axes[1,0].plot(df_old['X(m)'], df_old['Y(m)'], 'b--', label='Model Konstan', alpha=0.7)
    axes[1,0].plot(df_new['X(m)'], df_new['Y(m)'], 'r-', label='Model Environment', linewidth=2)
    axes[1,0].set_xlabel('Range (m)')
    axes[1,0].set_ylabel('Altitude (m)')
    axes[1,0].set_title('Trajectory Comparison')
    axes[1,0].legend()
    axes[1,0].grid(True, alpha=0.3)
    
    # 4. Density vs Altitude (untuk verifikasi)
    alts = np.linspace(0, 20000, 100)
    rho_isa = [1.225 * np.exp(-h/8400) for h in alts]  # Eksponensial
    rho_env = [Environment.getDensity(h) for h in alts]  # Model ISA berlapis (Environment.hpp)
    
    axes[1,1].plot(alts/1000, rho_isa, 'g--', label='Eksponensial (scale height)', alpha=0.7)
    axes[1,1].plot(alts/1000, rho_env, 'r-', label='Model Environment (ISA berlapis)', linewidth=2)
    axes[1,1].set_xlabel('Altitude (km)')
    axes[1,1].set_ylabel('Density (kg/m³)')
    axes[1,1].set_title('Atmosphere Model')
    axes[1,1].legend()
    axes[1,1].grid(True, alpha=0.3)
    axes[1,1].set_yscale('log')
    
    plt.tight_layout()
    plt.savefig('model_comparison.png', dpi=150)
    plt.show()
    
except FileNotFoundError:
    print("Jalankan simulasi dulu untuk menghasilkan kedua file CSV")