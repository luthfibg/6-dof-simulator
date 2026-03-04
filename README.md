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
- Mencetak status berkala (waktu, ketinggian, kecepatan, massa, dll.)
- Mengekspor data lintasan ke:
  - `trajectory.csv`
  - `trajectory.json`

File CSV berisi kolom:

```text
Time(s),X(m),Y(m),Z(m),Vx(m/s),Vy(m/s),Vz(m/s),
Roll(rad),Pitch(rad),Yaw(rad),Mass(kg),Thrust(N),Mach,Altitude(m)
```

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

### Fitur Utama (Level 4–5)

- **Model Atmosfer Dinamis (ISA)**:
  - Temperatur, tekanan, dan densitas berubah berdasarkan altitude
  - Menggunakan model International Standard Atmosphere (ISA) berlapis
  - Troposfer (0–11 km): temperatur turun linear dengan lapse rate
  - Tropopause/Stratosfer (11–20 km): model isotermal & eksponensial
  - Speed of sound dihitung dari temperatur lokal

- **Gravitasi Dinamis**:
  - Gravitasi berkurang dengan altitude: `g(h) = g₀ × (R/(R+h))²`
  - Menggunakan inverse square law dengan radius bumi R = 6.371 km

- **Model Massa yang Akurat**:
  - Pemisahan `fuelMass` (bahan bakar) dan `dryMass` (struktur rudal)
  - Hanya bahan bakar yang terbakar, massa struktur tetap
  - Menghindari bug massa → 0 yang menyebabkan percepatan tak terhingga

- **Thrust Vectoring**:
  - Arah thrust mengikuti orientasi rudal (Euler angles: pitch, yaw)
  - `F_thrust = T × [cos(pitch)cos(yaw), sin(pitch), cos(pitch)sin(yaw)]`
  - Tidak lagi fixed horizontal seperti versi sebelumnya

- **Ground Impact Detection**:
  - Simulasi berhenti otomatis saat rudal impact dengan tanah (Y ≤ 0)
  - Interpolasi linear untuk menentukan titik impact yang akurat

- **Fisika Dasar**:
  - Hukum Newton untuk translasi: `F = ma`
  - Hukum Euler untuk rotasi: `M = Iα`
  - Gaya aerodinamika (drag & lift) dengan koefisien `Cd0`, `CLa`, `Cm0`, `Cma`
  - Integrasi numerik Euler & Runge–Kutta orde 4 (RK4)

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

### Hasil Simulasi Realistis

Dengan konfigurasi default (main.cpp):
- **Initial conditions**: 500 kg (200 kg fuel), V₀ = 200 m/s horizontal + 50 m/s vertical
- **Propulsion**: 7.9s burn time, 5000N peak thrust
- **Hasil**:
  - Max altitude: ~487 m
  - Max speed: ~237 m/s (Mach 0.7)
  - Flight time: ~20 s (hingga impact)
  - Range: ~3.6 km

### Penyederhanaan yang Masih Ada

- Transformasi koordinat dari *inertial frame* ke *body frame* masih sederhana
- Koefisien aerodinamika konstan (belum fungsi Mach number)
- Model momen inersia sederhana (belum tensor inersia penuh)
- Belum ada sistem guidance/autopilot

### Pengembangan Selanjutnya

Repo ini cocok sebagai dasar untuk:

- Eksperimen dengan model aerodinamika non-linear (fungsi Mach, Reynolds)
- Implementasi kontrol guidance (proportional navigation, PID)
- Penambahan autopilot dengan fin deflection
- Simulasi multi-stage rocket
- Integrasi dengan real-time visualization (OpenGL/VTK)

---

## Lisensi

Tambahkan lisensi yang Anda inginkan di sini (misalnya MIT, BSD, dsb.).
