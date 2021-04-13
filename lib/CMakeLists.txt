# JetsonGPIO
add_library(JetsonGPIO
        JetsonGPIO/src/JetsonGPIO.cpp
        JetsonGPIO/src/PythonFunctions.cpp
        JetsonGPIO/src/gpio_pin_data.cpp)
target_include_directories(JetsonGPIO PUBLIC JetsonGPIO/include)