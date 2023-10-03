cmake_minimum_required (VERSION 3.21)

# Make MSVC use a statically-linked, multi-threaded runtime library
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

project ("Lidar_test")

# Add source to this project's executable.
include_directories(include)
add_executable (${PROJECT_NAME} "Lidar_test.cpp")
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

# Link Dependency
add_library(rplidar STATIC IMPORTED)

# RPLidar SDK installation path
set(RPLIDAR_SDK_PATH "C:/MYDATA/source/repos/rplidar_sdk")
set_property(TARGET rplidar PROPERTY IMPORTED_LOCATION 
  ${RPLIDAR_SDK_PATH}/output/win32/Debug/rplidar_driver.lib)

target_include_directories(rplidar INTERFACE
  ${RPLIDAR_SDK_PATH}/sdk/include/
  ${RPLIDAR_SDK_PATH}/sdk/src/)
target_link_libraries(${PROJECT_NAME} rplidar)
