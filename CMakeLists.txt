cmake_minimum_required(VERSION 3.10)
project(Qi_Table)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV. Jetson Nano has OpenCV 4.1.1 installed. Here minimal version 4 is required.
find_package(OpenCV 4 REQUIRED)
message("Found OpenCV")
message("    Version: " ${OpenCV_VERSION})
message("    Libraries: " ${OpenCV_LIBRARIES})
message("    Includes: " ${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Find boost
find_package(Boost REQUIRED system)

# Compile lib from source
add_subdirectory(lib)

# Header search path for all targets
include_directories(include)

add_executable(QiTable
        src/Control.cpp
        src/Vision.cpp
        src/Camera.cpp
        src/DeviceManager.cpp
        src/ChargerManager.cpp
        src/main.cpp)

add_executable(VisionUnitTest
        src/DeviceManager.cpp
        src/Camera.cpp
        src/Vision.cpp)

add_executable(GrabberUnitTest
        src/GrabberController.cpp
        tests/GrabberUnitTest.cpp)

target_link_libraries(QiTable ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} JetsonGPIO)
target_link_libraries(VisionUnitTest ${OpenCV_LIBS} pthread)
target_link_libraries(GrabberUnitTest ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})