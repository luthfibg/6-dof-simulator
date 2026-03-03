# Kumpulkan semua file sumber
file(GLOB_RECURSE SRC_FILES "*.cpp")
file(GLOB_RECURSE HEADER_FILES "*.h" "*.hpp")

# Buat library statis
add_library(core_sim_lib STATIC ${SRC_FILES} ${HEADER_FILES})

# Beri tahu compiler di mana mencari header
target_include_directories(core_sim_lib
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Jika Anda sudah menambahkan Eigen
if(EXISTS ${PROJECT_SOURCE_DIR}/lib/eigen)
    target_include_directories(core_sim_lib 
        PUBLIC 
            ${PROJECT_SOURCE_DIR}/lib/eigen
    )
endif()