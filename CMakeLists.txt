cmake_minimum_required(VERSION 3.22)

project (RPLidar_SFML)
set(SOURCES "RP-Lidar_Sfml.cpp")

message (STATUS "Fetching SFML..")
set(SFML_BUILD_NETWORK SFML_BUILD_AUDIO FALSE)
include(FetchContent)
FetchContent_Declare(
	SFML URL "https://github.com/SFML/SFML/archive/2.5.1.zip"
)
FetchContent_MakeAvailable(SFML)

# Make MSVC use a statically-linked, multi-threaded runtime library
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Add source to this project's executable.
add_executable(${PROJECT_NAME} ${SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

# Link Dependency: RPLidar-SDK is an imported static full library 
add_library(rplidar STATIC IMPORTED)

# The location of the library root folder in a Windows filesystem
set(RPLIDAR_SDK_PATH "C:\\Slamtec\\rplidar_sdk")
# The location of the headers
target_include_directories(rplidar INTERFACE ${RPLIDAR_SDK_PATH}/sdk/include/)
# The location of the library object by build type
set_target_properties(rplidar PROPERTIES 
	IMPORTED_LOCATION_DEBUG ${RPLIDAR_SDK_PATH}/output/win32/Debug/rplidar_driver${CMAKE_IMPORT_LIBRARY_SUFFIX}
	IMPORTED_LOCATION_RELEASE ${RPLIDAR_SDK_PATH}/output/win32/Release/rplidar_driver${CMAKE_IMPORT_LIBRARY_SUFFIX})

# Link the target executable to both libs
target_link_libraries(${PROJECT_NAME} PRIVATE rplidar sfml-graphics)

# copy all of the DLLs that a target depends on into its output directory 
if (WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD  # Adds a post-build event
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
		$<TARGET_FILE_DIR:${PROJECT_NAME}> COMMAND_EXPAND_LISTS)
endif()
# or
#install(TARGETS ${PROJECT_NAME} RUNTIME)
#install(FILES $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> TYPE BIN)