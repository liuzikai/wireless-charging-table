//
// Created by liuzikai on 4/15/21.
//

#include "ChargerManager.h"
#include <iostream>

std::unique_ptr<ChargerManager> chargerManager;

int main() {

    chargerManager = std::make_unique<ChargerManager>();

    while (true) {
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++) {
            std::cout << "i: ";
            switch (chargerManager->getChargerStatus(i)) {
                case ChargerManager::UNKNOWN:
                    std::cout << "UNKNOWN         ";
                    break;
                case ChargerManager::NOT_CHARGING:
                    std::cout << "NOT_CHARGING    ";
                    break;
                case ChargerManager::CHARGING:
                    std::cout << "CHARGING        ";
                    break;
            }
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}