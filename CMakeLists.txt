cmake_minimum_required(VERSION 3.0.0)
project(Drone)

set (CMAKE_CXX_STANDARD 14)

# Include the Necessary Headers
include_directories(./
    ./Controls
    ./Sensors
    ./Utils
    ./Indirect
)


# Add the Linker Directories
#link_directories()

# Some global configurations
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Build our Executable
set(BASE_SOURCES ./Drone.cpp 
	./ManualController.cpp
    ./Controls/DirectControls.cpp
    ./Sensors/rpcSensors.cpp
    ./Indirect/SerialRX.cpp
)

set(CMAKE_CXX_FLAGS "-O2 -fpic")
set(CMAKE_C_FLAGS "-O2 -fpic")

add_library(${PROJECT_NAME} SHARED ${BASE_SOURCES})
#add_executable(${PROJECT_NAME} ${BASE_SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)
#target_link_libraries(${PROJECT_NAME} rpc)

#install(TARGETS $(PROJECT_NAME) DESTINATION lib)