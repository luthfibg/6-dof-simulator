# Buat executable untuk aplikasi utama
add_executable(missile_sim main.cpp)

# Tautkan dengan library kita
target_link_libraries(missile_sim PRIVATE core_sim_lib)

# Beri tahu di mana mencari header
target_include_directories(missile_sim PRIVATE ${PROJECT_SOURCE_DIR}/src)