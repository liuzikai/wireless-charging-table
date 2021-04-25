cmake_minimum_required(VERSION 3.10)
project(Qi_Table)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV. Jetson Nano has OpenCV 4.1.1 installed. Here minimal version 4 is required.
find_package(OpenCV 4 REQUIRED)
message("Found OpenCV")
message("    Version: " ${OpenCV_VERSION})
message(VERBOSE "    Libraries: " ${OpenCV_LIBRARIES})
message(VERBOSE "    Includes: " ${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Find boost
find_package(Boost REQUIRED system filesystem)

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
        src/GrabberController.cpp
        src/main.cpp)

add_executable(VisionUnitTest
        src/DeviceManager.cpp
        src/Camera.cpp
        src/Vision.cpp
        tests/VisionUnitTest.cpp)

add_executable(ParameterTuningTest
        tests/ParameterTuning.cpp)

add_executable(GrabberUnitTest
        src/GrabberController.cpp
        tests/GrabberUnitTest.cpp)

add_executable(ChargerManagerUnitTest
        src/ChargerManager.cpp
        tests/ChargerManagerUnitTest.cpp)

add_executable(GPIOUnitTest
        tests/GPIOUnitTest.cpp)

add_executable(ImageCalibrationTest
        tests/imageCalibration.cpp)

target_link_libraries(QiTable ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} JetsonGPIO pthread)
target_link_libraries(VisionUnitTest ${OpenCV_LIBS} pthread)
target_link_libraries(ParameterTuningTest ${OpenCV_LIBS} pthread)
target_link_libraries(ImageCalibrationTest ${OpenCV_LIBS} pthread)
target_link_libraries(GrabberUnitTest ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} pthread)
target_link_libraries(ChargerManagerUnitTest JetsonGPIO pthread)
target_link_libraries(GPIOUnitTest JetsonGPIO pthread)