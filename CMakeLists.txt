cmake_minimum_required(VERSION 3.22)

project (Lidar_PC_SFML)
set(SOURCES "Lidar_test.cpp")

message (STATUS "Fetching SFML..")
set(SFML_BUILD_NETWORK SFML_BUILD_AUDIO FALSE)
include(FetchContent)
FetchContent_Declare(
	SFML
	URL "https://github.com/SFML/SFML/archive/2.5.1.zip"
)
FetchContent_MakeAvailable(SFML)

# Make MSVC use a statically-linked, multi-threaded runtime library
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Add source to this project's executable.
add_executable(${PROJECT_NAME} #[[WIN32]] ${SOURCES})
#  WIN32 flag: Removes Console window and makes it a Win32 application with WinMain entry point 
#set_target_properties(target PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>) # same for release

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG)
## comment out the above to enable debug logging

# Link Dependency
add_library(rplidar STATIC IMPORTED)

# RPLidar SDK installation path
set(RPLIDAR_SDK_PATH "C:/MYDATA/source/repos/rplidar_sdk")
set_property(TARGET rplidar PROPERTY IMPORTED_LOCATION 
  ${RPLIDAR_SDK_PATH}/output/win32/Debug/rplidar_driver.lib)

target_include_directories(rplidar INTERFACE
  ${RPLIDAR_SDK_PATH}/sdk/include/
  ${RPLIDAR_SDK_PATH}/sdk/src/)

target_link_libraries(${PROJECT_NAME} PRIVATE rplidar sfml-graphics)

# copy all of the DLLs that a target depends on into its output directory 
if (WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy 
		$<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> 
		$<TARGET_FILE_DIR:${PROJECT_NAME}>
		COMMAND_EXPAND_LISTS)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy 
		${FETCHCONTENT_BASE_DIR}/sfml-src/extlibs/bin/x64/openal32.dll 
		$<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif()