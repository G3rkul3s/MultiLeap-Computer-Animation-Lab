cmake_minimum_required(VERSION 3.29)
# cmake .. -G"Unix Makefiles"
project(Lab)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LeapSDK_PATH "C:/Program Files/Ultraleap/LeapSDK")

find_package(LeapSDK 5 PATHS "${LeapSDK_PATH}/lib/cmake/LeapSDK" REQUIRED)

# Include directories
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/src")

# Add source files
add_executable(Lab
    src/save_devices_data_structure.cpp
    src/vecm.cpp
    src/cursor_manips.cpp
    src/main.cpp
    src/pch.cpp
    src/funcs.cpp
)

# Link LeapSDK
target_link_libraries(Lab PUBLIC LeapSDK::LeapC)
# Include LeapSDK headers
target_include_directories(Lab PUBLIC "${PROJECT_SOURCE_DIR}/include")

# Include Eigen headers
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0")
target_include_directories(Lab PUBLIC ${EIGEN3_INCLUDE_DIR})

# Copy LeapC.dll after build
add_custom_command(TARGET Lab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${LeapSDK_PATH}/lib/x64/LeapC.dll" $<TARGET_FILE_DIR:Lab>
)

# LeapSDK includes
target_include_directories(Lab PUBLIC "${LeapSDK_PATH}/include")

# Add precompiled headers
target_precompile_headers(Lab PRIVATE include/pch.h)