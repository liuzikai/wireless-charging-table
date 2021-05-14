//
// Created by liuzikai on 4/13/21.
//

#ifndef QI_TABLE_CHARGERMANAGER_H
#define QI_TABLE_CHARGERMANAGER_H

#include "Common.h"
#include <thread>
#include <atomic>
#include <chrono>

class ChargerManager {
public:

    explicit ChargerManager();

    ~ChargerManager();

    enum Status {
        UNKNOWN,
        NOT_CHARGING,  // including idle and blinking (foreign object detected)
        CHARGING,
    };

    /**
     * Get the status of a charger
     * @param id  The charger index, must be in [0, CHARGER_COUNT)
     * @return    The charger status
     */
    Status getChargerStatus(int id) const { return status[id]; }

    /**
     * The number of charger
     */
    static constexpr unsigned CHARGER_COUNT = 3;

    /**
     * Wait for this time if the status is UNKNOWN
     */
    static constexpr unsigned UNKNOWN_STATUS_WAIT_TIME_MS = 500;  // [ms]

private:

    std::thread updateThread;
    std::atomic<bool> threadShouldExit;
    void update();

    Status status[CHARGER_COUNT] = {UNKNOWN};

    int red[CHARGER_COUNT] = {0};
    int green[CHARGER_COUNT] = {0};

    // The pin numbers for coil status input in BCM mode (see the datasheet)
    static constexpr int RED_PINS[CHARGER_COUNT] = {6, 9, 22};
    static constexpr int GREEN_PINS[CHARGER_COUNT] = {5, 10, 27};

    static constexpr unsigned UPDATE_INTERVAL_MS = 100;  // [ms]

};

#endif //QI_TABLE_CHARGERMANAGER_H
