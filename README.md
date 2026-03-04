# Missile 6-DoF Simulator

Simulasi numerik sederhana untuk lintasan rudal dengan model 6-DoF (6 degrees of freedom). Proyek ini ditulis dalam C++ menggunakan CMake dan Eigen, dengan visualisasi hasil menggunakan Python (pandas + matplotlib).

Fokus utama repositori ini:

- Membangun *core library* fisika rudal (`core_sim_lib`)
- Menyediakan aplikasi CLI sederhana (`missile_sim`) untuk menjalankan simulasi
- Mengekspor data lintasan ke CSV/JSON
- Memvisualisasikan lintasan 2D & 3D menggunakan `visualize.py`
- Menyediakan unit test dasar untuk `Vector3D` dan dinamika rudal

---

## Struktur Proyek

```text
CMakeLists.txt           # Root CMake project
apps/
  main.cpp              # Aplikasi utama: menjalankan simulasi
src/
  CMakeLists.txt
  core/
    Vector3D.*          # Kelas vektor 3D utilitas
    Missile.*           # State rudal + perhitungan gaya/momen
    Simulation.*        # Loop simulasi & ekspor data
    MissileDynamics.*   # Persamaan diferensial (Hukum Newton & Euler)
    StateDerivatives.*  # Struct turunan state (dx/dt)
    NumericalIntegrator.hpp  # Integrator RK4 & Euler
    Environment.hpp     # Model atmosfer ISA & gravitasi dinamis
    ControlSurface.*    # Representasi fisik sirip kendali (elevator, rudder, aileron)
    Autopilot.*         # PID controller untuk attitude rudal
    Guidance.*          # Algoritma guidance (Pure Pursuit, Proportional Navigation)

lib/
  eigen/                # Eigen (header-only linear algebra)

tests/
  test_vector3d.cpp     # Unit test untuk Vector3D
  test_missile.cpp      # Unit test dasar rudal

visualize.py            # Script Python untuk plotting lintasan
compare_models.py       # Script untuk membandingkan hasil simulasi
Environment.py          # Modul Python untuk model atmosfer (mirror dari C++)
```

---

## Prasyarat

- C++ compiler dengan dukungan C++17
- CMake ≥ 3.15
- Eigen (sudah disertakan di `lib/eigen`)
- Python 3.12 (atau 3.x lain) untuk visualisasi
- Library Python:
  - `pandas`
  - `matplotlib`
  - `numpy`

Disarankan menggunakan **VS Code + CMake Tools** atau **CMake + Ninja / Visual Studio**.

---

## Build (C++)

Dari root proyek:

```bash
cmake -S . -B build
cmake --build build
```

Atau dengan konfigurasi `Release`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Jika memakai VS Code + CMake Tools:

1. Buka folder ini di VS Code
2. Pilih kit/toolchain C++ (MSVC / MinGW / clang)
3. `CMake: Configure`
4. `CMake: Build`

Hasil build utama:

- Library statis: `build/src/core_sim_lib.lib`
- Aplikasi simulasi: `build/apps/missile_sim.exe`
- Executable test: `build/tests/test_vector3d.exe`, `build/tests/test_missile.exe`

---

## Menjalankan Simulasi

Dari root proyek (bukan dari dalam `build/`):

```bash
./build/apps/missile_sim.exe
```

Program akan:

- Menjalankan simulasi selama `maxTime` (default 60 s, lihat `apps/main.cpp`)
- Menggunakan autopilot + guidance untuk mengejar target
- Mencetak status berkala (waktu, ketinggian, kecepatan, massa, dll.)
- Mengekspor data lintasan ke:
  - `trajectory.csv`
  - `trajectory.json`

File CSV berisi kolom:

```text
Time(s),X(m),Y(m),Z(m),Vx(m/s),Vy(m/s),Vz(m/s),
Roll(rad),Pitch(rad),Yaw(rad),Mass(kg),Thrust(N),Mach,Altitude(m)
```

### Mengatur Target & Guidance

Mode default: **Ballistic (Gravity Turn)** dengan target 200 km downrange.

```bash
./build/apps/missile_sim.exe                                           # Default: 200 km ballistic
./build/apps/missile_sim.exe --target 100000 0 0                       # 100 km downrange
./build/apps/missile_sim.exe --target 300000 0 0 --guidance ballistic  # 300 km ballistic
./build/apps/missile_sim.exe --target 8000 0 2000 --guidance pure      # Taktis: Pure Pursuit
./build/apps/missile_sim.exe --target 6000 0 3000 --guidance pn        # Taktis: Proportional Navigation
./build/apps/missile_sim.exe --target 200000 0 0 --launch-angle 82     # Manual launch angle override
```

`--guidance MODE`:

| Mode | Deskripsi |
|------|-----------|
| `ballistic` | Gravity turn boost → free-flight ballistic coast (default MRBM) |
| `pure` | Pure Pursuit — mengarah langsung ke posisi target |
| `pn` | Proportional Navigation — menggunakan LOS rate |

`--launch-angle DEG`: Override sudut peluncuran (derajat). Jika tidak diberikan, dihitung otomatis dari jarak target menggunakan model empiris.

Konvensi koordinat: `Vector3D(x, y, z)` — x: range, y: altitude, z: crossrange.

---

## Visualisasi dengan Python

Disarankan membuat virtual environment Python di root proyek:

```bash
py -3.12 -m venv .venv
# Aktivasi di PowerShell
.\.venv\Scripts\Activate.ps1

pip install pandas matplotlib numpy
```

Setelah simulasi dijalankan dan `trajectory.csv` terbentuk di root, jalankan:

```bash
python visualize.py
```

Script ini akan menghasilkan:

- `trajectory_plots.png` — beberapa plot 2D (X–Y, altitude vs time, speed vs time, Mach vs time)
- `trajectory_3d.png` — plot lintasan 3D (X, Y, Z)

---

## Menjalankan Unit Test

Setelah build dengan CMake (CTest sudah diaktifkan di `CMakeLists.txt` root), dari folder `build/`:

```bash
ctest          # untuk konfigurasi default (mis. Debug)
ctest -C Debug # jika multi-config (MSVC, dll.)
```

Test yang tersedia saat ini:

- `Vector3DTest` — operasi dasar vektor 3D
- `MissileTest` — skenario rudal sederhana

Anda juga dapat menjalankan executable test secara langsung:

```bash
./build/tests/test_vector3d.exe
./build/tests/test_missile.exe
```

---

## Catatan Model Fisika

Ini **bukan** model balistik tingkat produksi; tujuan utamanya adalah edukasi dan eksperimen numerik:

### Fitur Utama (Level 4–6: MRBM)

- **Spesifikasi Rudal: Rajawali-2 MRBM**:
  - Massa peluncuran: 6.500 kg (propelan: 4.000 kg, struktur+warhead: 2.500 kg)
  - Geometri: diameter 0.88 m, panjang 11.0 m
  - Propulsi: motor roket cair, Isp ≈ 230 s, peak thrust 210 kN, burn time 65 s
  - Total impulse: 9.1 MN·s
  - Aerodinamika: Cd0 = 0.12, CLα = 1.8, Cmα = -0.3
  - Jangkauan desain: 100 – 320 km

- **Model Atmosfer 7-Lapis (ISA Extended)**:
  - Troposfer (0–11 km): lapse rate -6.5°C/km
  - Tropopause (11–20 km): isotermal 216.65 K
  - Stratosfer bawah (20–32 km): +1°C/km
  - Stratosfer atas (32–47 km): +2.8°C/km
  - Stratopause (47–51 km): isotermal 270.65 K
  - Mesosfer (51–86 km): -2.8°C/km
  - Termosfer dasar (>86 km): +2°C/km
  - Density clamp untuk near-vacuum di ketinggian ekstrem

- **Gravitasi Dinamis**:
  - `g(h) = g₀ × (R/(R+h))²` (inverse-square law)

- **Model Thrust Trapezoidal**:
  - Ignisi: 30% thrust pada t=0, ramp ke 100% dalam 0.5 s
  - Steady burn pada thrust konstan
  - Shutdown ramp: 100% → 0% dalam 0.5 s terakhir
  - Konsumsi bahan bakar proporsional dengan thrust sesaat

- **Thrust Vector Control (TVC)**:
  - Jet vane TVC aktif selama boost phase
  - 3% efisiensi thrust → lateral force per radian defleksi
  - Moment arm: 45% panjang rudal
  - Memberikan kontrol attitude di ketinggian tinggi (rendah tekanan dinamis)

- **AoA Stall Protection**:
  - Angle of attack efektif dibatasi ±15° dalam perhitungan gaya & momen aerodinamika
  - Mencegah drag berlebihan akibat AoA besar saat gravity turn

- **Ballistic Guidance (Gravity Turn)**:
  - Fase 0–3 s: tahan sudut peluncuran (stabilisasi awal)
  - Fase boost (3 s – burnout): gravity turn (α ≈ 0, mengikuti vektor kecepatan)
  - Fase coast/reentry: free-flight balistik
  - Sudut peluncuran dihitung otomatis dari jarak target (model empiris kalibrasi)

### Fitur Level 5: Autopilot & Guidance

- **ControlSurface (Sirip Kendali)**:
  - Mewakili elevator, rudder, aileron dengan defleksi terbatas (±20°)
  - Menghasilkan momen tambahan pada rudal berdasarkan defleksi dan dynamic pressure

- **Autopilot (PID Attitude Controller)**:
  - Kontrol PID per sumbu (roll, pitch, yaw)
  - Menghitung defleksi elevator/rudder/aileron agar orientasi rudal mengikuti orientasi yang diinginkan
  - Menyertakan anti-windup sederhana pada integral

- **Guidance (Pemandu Target)**:
  - `purePursuit(missilePos, missileVel)`: mengarahkan rudal langsung ke posisi target
  - `proportionalNavigation(...)`: kerangka awal PN dengan gain navigasi `N`
  - Menghasilkan `desiredOrientation` (roll, pitch, yaw) yang menjadi referensi autopilot

- **Integrasi dengan Missile & Simulation**:
  - `Missile` menyimpan daftar control surface dan menambahkan momen dari tiap sirip ke dinamika
  - Di `apps/main.cpp`, setiap langkah simulasi:
    - Guidance menghitung `desiredOrientation` menuju target
    - Autopilot menghitung defleksi sirip berdasarkan error orientasi
    - `Missile::setControlDeflections()` menerapkan defleksi ke elevator/rudder/aileron

### Hasil Simulasi MRBM Rajawali-2

| Target | Launch | Apogee | V_max | T_flight | Impact | Error |
|--------|--------|--------|-------|----------|--------|-------|
| 100 km | 74.8° | 7.4 km | Mach 5.5 | 107 s | 107 km | +7% |
| 150 km | 77.0° | 15.3 km | Mach 5.7 | 147 s | 147 km | -2% |
| 200 km | 79.2° | 22.0 km | Mach 5.7 | 173 s | 209 km | +5% |
| 300 km | 83.8° | 68.1 km | Mach 5.5 | 295 s | 318 km | +6% |

Catatan: sudut peluncuran dihitung otomatis. Akurasi ±7% tanpa terminal guidance.
Gunakan `--launch-angle` untuk override manual.

### Penyederhanaan yang Masih Ada

- Koefisien aerodinamika konstan (belum fungsi Mach number/Reynolds)
- Model momen inersia sederhana (belum tensor inersia penuh)
- Bumi datar (belum memperhitungkan kelengkungan bumi untuk jarak >300 km)
- Belum ada terminal guidance (homing seeker) untuk presisi impact
- AoA stall model: hard clamp, belum ada drag rise gradual
- Wind/turbulence belum dimodelkan

### Pengembangan Selanjutnya

- Model aerodinamika non-linear (Cd = f(Mach, AoA))
- Terminal guidance (radar/IR seeker)
- Rotating Earth & WGS-84 ellipsoid untuk jarak >500 km
- Multi-stage rocket separation
- Monte Carlo dispersi & CEP analysis
- Real-time 3D visualization (OpenGL/VTK)
- Hardware-in-the-loop (HITL) interface

---

## Lisensi

Tambahkan lisensi yang Anda inginkan di sini (misalnya MIT, BSD, dsb.).
