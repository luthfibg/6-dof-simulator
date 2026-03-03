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

lib/
  eigen/                # Eigen (header-only linear algebra)

tests/
  test_vector3d.cpp     # Unit test untuk Vector3D
  test_missile.cpp      # Unit test dasar rudal

visualize.py            # Script Python untuk plotting lintasan
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

- Mengimplementasikan hukum Newton dan hukum Euler untuk translasi dan rotasi
- Menggunakan koefisien aerodinamika sederhana (`Cd0`, `CLa`, `Cm0`, `Cma`)
- Dinamika massa rudal (massa berkurang selama pembakaran propelan)
- Integrasi numerik dengan metode Euler dan Runge–Kutta orde 4 (RK4) melalui `NumericalIntegrator`

Beberapa penyederhanaan yang masih ada:

- Atmosfer dianggap konstan (ρ tetap)
- Transformasi dari gaya di *inertial frame* ke *body frame* masih disederhanakan
- Model thrust dan lift masih bentuk sederhana (parabola/linear)

Repo ini cocok sebagai dasar untuk:

- Latihan numerik persamaan diferensial
- Eksperimen dengan model aerodinamika yang lebih kaya
- Penambahan kontrol guidance / autopilot di masa depan

---

## Lisensi

Tambahkan lisensi yang Anda inginkan di sini (misalnya MIT, BSD, dsb.).
