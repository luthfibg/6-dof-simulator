import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Baca data
df = pd.read_csv('trajectory.csv')

# Buat figure dengan subplots
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))

# Plot 1: Trajectory 2D (X vs Y)
ax1.plot(df['X(m)'], df['Y(m)'], 'b-', linewidth=1)
ax1.set_xlabel('Range (m)')
ax1.set_ylabel('Altitude (m)')
ax1.set_title('Missile Trajectory')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)

# Plot 2: Altitude vs Time
ax2.plot(df['Time(s)'], df['Altitude(m)'], 'r-', linewidth=1)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')
ax2.set_title('Altitude vs Time')
ax2.grid(True, alpha=0.3)

# Plot 3: Velocity vs Time
ax3.plot(df['Time(s)'], np.sqrt(df['Vx(m/s)']**2 + 
                                 df['Vy(m/s)']**2 + 
                                 df['Vz(m/s)']**2), 'g-', linewidth=1)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Speed (m/s)')
ax3.set_title('Speed vs Time')
ax3.grid(True, alpha=0.3)

# Plot 4: Mach Number vs Time
ax4.plot(df['Time(s)'], df['Mach'], 'm-', linewidth=1)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Mach Number')
ax4.set_title('Mach Number vs Time')
ax4.grid(True, alpha=0.3)
ax4.axhline(y=1, color='k', linestyle='--', alpha=0.5, label='Mach 1')
ax4.legend()

plt.tight_layout()
plt.savefig('trajectory_plots.png', dpi=150)
plt.show()

# 3D Plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory
ax.plot(df['X(m)'], df['Z(m)'], df['Y(m)'], 'b-', linewidth=1)

# Plot start point
ax.scatter(df['X(m)'].iloc[0], df['Z(m)'].iloc[0], df['Y(m)'].iloc[0], 
          color='g', s=100, label='Start')

# Plot end point
ax.scatter(df['X(m)'].iloc[-1], df['Z(m)'].iloc[-1], df['Y(m)'].iloc[-1], 
          color='r', s=100, label='End')

ax.set_xlabel('X (m)')
ax.set_ylabel('Z (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('3D Missile Trajectory')
ax.legend()

plt.tight_layout()
plt.savefig('trajectory_3d.png', dpi=150)
plt.show()

print("Visualisasi selesai! Plot tersimpan sebagai PNG.")