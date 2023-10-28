cmake_minimum_required(VERSION 3.26.4)

project (RPLidar_SFML)

message (STATUS "Fetching SFML..")
include(FetchContent)
FetchContent_Declare( SFML  
	GIT_REPOSITORY https://github.com/SFML/SFML.git GIT_TAG 2.6.x
	#URL "https://github.com/SFML/SFML/archive/2.6.0.zip"
)
FetchContent_MakeAvailable(SFML)

# Add source to this project's executable.
set(SOURCES "RP-Lidar_Sfml.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
set(SFML_BUILD_NETWORK SFML_BUILD_AUDIO FALSE)

# when targeting MSVC select a multithreaded, statically linked runtime
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	set_property(TARGET ${PROJECT_NAME} PROPERTY
		MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Link Dependency: RPLidar-SDK is an imported static full library 
add_library(rplidar STATIC IMPORTED)

# The location of the sdk root, on windows or linux
if(WIN32)
	set(RPLIDAR_SDK_PATH "C:\\Slamtec\\rplidar_sdk")
else()
	set(RPLIDAR_SDK_PATH "/usr/lib/Slamtec/rplidar_sdk")
endif()
# Location of the headers, and executable by configuration
set_target_properties(rplidar PROPERTIES 
	INTERFACE_INCLUDE_DIRECTORIES "${RPLIDAR_SDK_PATH}/sdk/include/"
	IMPORTED_LOCATION_DEBUG ${RPLIDAR_SDK_PATH}/output/win32/Debug/rplidar_driver${CMAKE_IMPORT_LIBRARY_SUFFIX}
	IMPORTED_LOCATION_RELEASE ${RPLIDAR_SDK_PATH}/output/win32/Release/rplidar_driver${CMAKE_IMPORT_LIBRARY_SUFFIX}
)
# Link the target executable to both libs
target_link_libraries(${PROJECT_NAME} PRIVATE rplidar sfml-graphics)

# copy all of the DLLs that a target depends on into its output directory 
if (WIN32)
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
			$<TARGET_FILE_DIR:${PROJECT_NAME}>
		COMMAND_EXPAND_LISTS)
endif()
# or
#install(TARGETS ${PROJECT_NAME} RUNTIME_DEPENDENCIES <TARGET_FILE_DIR:${PROJECT_NAME}> TYPE BIN)