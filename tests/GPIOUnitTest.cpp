//
// Created by liuzikai on 4/24/21.
//

#include <thread>
#include <chrono>
#include <iostream>
#include <JetsonGPIO.h>

static constexpr int PINS[] = {9, 10};

int main() {

    GPIO::setmode(GPIO::BCM);

    for (int pin : PINS) {
        GPIO::setup(pin, GPIO::Directions::IN);
    }

    while(true) {

        for (int pin : PINS) {
            std::cout << "[" << pin << "]" << GPIO::input(pin) << "    ";
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;

}