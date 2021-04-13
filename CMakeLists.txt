cmake_minimum_required(VERSION 3.17)
project(Qi_Table)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV. Jetson Nano has OpenCV 4.1.1 installed. Here minimal version 4 is required.
find_package(OpenCV 4 REQUIRED)
message("Found OpenCV")
message("    Version: " ${OpenCV_VERSION})
message("    Libraries: " ${OpenCV_LIBRARIES})
message("    Includes: " ${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(QiTable
        Control.cpp
        Vision.cpp
        Camera.cpp
        Device_manager.cpp
        main.cpp)

add_executable(VisionUnitTest
        Camera.cpp
        Device_manager.cpp
        Vision.cpp)
add_executable(mytest
               test.cpp)
# link OpenCV library correctly.
target_link_libraries(mytest ${OpenCV_LIBS})
target_link_libraries(QiTable ${OpenCV_LIBS})
target_link_libraries(VisionUnitTest ${OpenCV_LIBS})