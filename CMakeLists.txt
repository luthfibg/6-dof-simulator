cmake_minimum_required(VERSION 3.15)
# Nama proyek Anda. Ini akan menjadi top-level.
project(Missile6DoF LANGUAGES CXX)

# Set standar C++ (gunakan 17, 20, atau 14 sesuai kebutuhan)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Opsi untuk build jenis Debug/Release
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Untuk editor seperti VSCode

# Aktifkan CTest untuk menjalankan unit test
enable_testing()

# Tambahkan sub-direktori. Urutan bisa penting jika ada dependensi.
# Jika lib/ hanya berisi Eigen (header-only), kita tidak perlu menambahkannya di sini.
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(apps)