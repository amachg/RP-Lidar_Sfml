cmake_minimum_required(VERSION 3.26.4)

project (RPLidar_SFML)

message (STATUS "Fetching SFML..")
include(FetchContent)
FetchContent_Declare( SFML 
	# URL of the git repository and Git branch name, tag or commit hash. 
	#GIT_REPOSITORY https://github.com/SFML/SFML.git GIT_TAG 2.6 # or master

	# List of paths and/or URL(s) of the external project's source.
	URL "https://github.com/SFML/SFML/archive/2.6.0.zip"
)
FetchContent_MakeAvailable(SFML)

# Select a multi-threaded statically-linked MSVC runtime library for MSVCompiler.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Add source to this project's executable.
set(SOURCES "RP-Lidar_Sfml.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
set(SFML_BUILD_NETWORK SFML_BUILD_AUDIO FALSE)

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