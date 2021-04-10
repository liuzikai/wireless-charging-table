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


add_executable(QiTable
        Control.cpp
        Vision.cpp
        main.cpp)

add_executable(VisionUnitTest
        Vision.cpp)

target_link_libraries(QiTable ${OpenCV_LIBS})
target_link_libraries(VisionUnitTest ${OpenCV_LIBS})