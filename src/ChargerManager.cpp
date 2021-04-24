//
// Created by liuzikai on 4/13/21.
//

#include "ChargerManager.h"
#include <iostream>
#include <JetsonGPIO.h>

constexpr unsigned ChargerManager::UPDATE_INTERVAL_MS;
constexpr unsigned ChargerManager::CHARGER_COUNT;
constexpr int ChargerManager::RED_PINS[ChargerManager::CHARGER_COUNT];
constexpr int ChargerManager::GREEN_PINS[ChargerManager::CHARGER_COUNT];
constexpr unsigned ChargerManager::UPDATE_INTERVAL_MS;


ChargerManager::ChargerManager() {
    GPIO::setmode(GPIO::NumberingModes::BCM);
    threadShouldExit = false;
    updateThread = std::thread(&ChargerManager::update, this);
}

ChargerManager::~ChargerManager() {
    threadShouldExit = true;
    updateThread.join();
}

void ChargerManager::update() {
    while (!threadShouldExit) {

        for (int i = 0; i < CHARGER_COUNT; i++) {

            int newRed = GPIO::input(RED_PINS[i]);
            int newGreen = GPIO::input(GREEN_PINS[i]);


            if (green[i] == 0 && newGreen == 1) {  // green 0 -> 1 indicates start charging

                // Sanity check: red should be off
                if (newRed == 0) {
                    status[i] = CHARGING;  // repeated setting to CHARGING due to green's breathing doesn't matter
                } else {
                    std::cerr << "Charger " << i << ": green 0 -> 1 but red is 1" << std::endl;
                    status[i] = UNKNOWN;
                }

            } else if (red[i] == 0 && newRed == 1) {  // red 0 -> 1 indicates stop charging

                // Sanity check: green should be off
                if (newGreen == 0) {
                    status[i] = NOT_CHARGING;  // repeated setting to NOT_CHARGING due to red's blinking doesn't matter
                } else {
                    std::cerr << "Charger " << i << ": red 0 -> 1 but green is 1" << std::endl;
                    status[i] = UNKNOWN;
                }

            }  // Otherwise, keep the current status

            red[i] = newRed;
            green[i] = newGreen;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_INTERVAL_MS));
    }
}
