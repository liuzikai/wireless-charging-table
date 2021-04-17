
// for delay function.
#include <chrono> 
#include <thread>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <JetsonGPIO.h>
#include <set>

#include "../include/Control.h"

#define ERROR_(message_) { \
    errorMessage = message_; \
    curState = ERROR; \
    return -1; }


using namespace std;


static pthread_t pin_thread[ChargerManager::CHARGER_COUNT];

// TODO: Get from vision
static volatile bool visionNeedsHandling;
static volatile vector<cv::Point> newDevices;
static volatile vector<cv::Point> removedDevices;

// static pthread_mutex_t pin_locks[ChargerManager::CHARGER_COUNT]; 


/********************************* Public Functions *****************************/

Control::Control(){
    
    curState = WAITING;

    // Fill the schedule functions table
    schedule[0] = &scheduleWaiting;
    schedule[1] = &scheduleCalculating;
    schedule[2] = &scheduleMoving1;
    schedule[3] = &scheduleMoving2;
    schedule[4] = &scheduleError;

    // Init the coil positions
    curCoilPositions[0] = cv::Point(0,0);
    curCoilPositions[1] = cv::Point(0,0);
    curCoilPositions[2] = cv::Point(0,0);
    idleCoilCount = ChargerManager::CHARGER_COUNT;


}

int Control::launch(){
    
    // Launch the thread for pin status checking
    for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
        if (0 != pthread_create(&pin_thread[i], NULL, GPIO_read_inputs, &i)){
            curState = ERROR;
            errorMessage = "Pin status thread creation failure";
        }
    }

    while(1){
        schedule[curState]();
        sleep(1); // Sleep for 1s
    }

    return 0; // Should never reach here
}


/********************************* Schduling Controls *****************************/

int scheduleWaiting(){
    
    bool needMoving = false;
    set<cv::Point> toIgnore; // filled when new status is CHARGING
    set<cv::Point> toConfirm; // filled when new status is NOT_CHARGING

    // Pull for the wireless charging status
    for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
        auto curStatus = chargerManager.getChargerStatus(i);
        
        // Retry for once if the status is unknown
        if (curStatus == ChargerManager::UNKNOWN) {
            usleep(500000);
            curStatus = chargerManager.getChargerStatus(i);
        }

        if (curStatus == ChargerManager::UNKNOWN){
            ERROR_("Pull charging status failed");
        }

        // Handle if there is a new status
        if (curStatus != oldStatus[i]){
            oldStatus[i] = curStatus;
            
            switch (curStatus){
                
                // A device is put at the coil
                case ChargerManager::CHARGING: 
                    // Let it go and update available coils
                    idleCoilCount--;
                    
                    // The new device position at this coil should be ignored later
                    if (chargeable.find(curCoilPositions[i]) != chargable.end()){
                        ERROR_("Duplicate Device!");
                    }
                    toIgnore.insert(curCoilPositions[i]);
                    chargeable[curCoilPositions[i]] = Device(curCoilPositions[i]);

                    break;

                // A device is removed/finished charging
                case ChargerManager::NOT_CHARGING: 
                    
                    idleCoilCount++;
                    toConfirm.insert(curCoilPositions[i]);
                    needMoving = true;
                    break;

                default: 
                    assert(0);
            }
        }
    }

    // TODO: Need a pause here for vision to update?

    // Pull for vision status
    if (visionNeedsHandling){
        for (const auto& newDevice : newDevices){
            // Check whether this new device should be ignored
            if (toIgnore.find(newDevice) != toIgnore.end()){
                toIgnore.erase(newDevice);
                continue;
            }

            toSechdule.insert(newDevice);

        }

        for (const auto& removedDevice : removedDevices){
            // Check the confirm: the removal has already been noticed by wireless charging
            if (toConfirm.find(removedDevice) != toConfirm.end()){
                
                toConfirm.erase(removedDevice);

                // The device was charging but taken away
                if (!chargeable.erase(removedDevice)){
                    ERROR_("Chargeable map panic");
                }

                continue;
            }

            // Remove the device that is either unchargeable or not scheduled
            if (!unchargeable.erase(removedDevice)){
                if (toSchedule.find(removedDevice) == toSchedule.end()){
                    ERROR_("Unchargeable map or to schedule panic");
                }
                
                toSchedule.erase(removedDevice);
                
            }
        }
    }

    visionNeedsHandling = false;

    if (toIgnore.size() != 0){
        ERROR_("Conflict message from wireless and vision!");
    }

    // Mark the remaining toConfirm devices as unchargeable (finished charging)
    for (const auto& confirm : toConfirm){
        auto curDevice = chargeable.find(confirm);
        
        if (curDevice != chargeable.end()){
            unchargeable.emplace(curDevice);
            chargeable.erase(curDevice.first);
        } else {
            ERROR_("Conflict message from wireless and vision!");
        }
    }

    needMoving |= toSchedule.size() > 0 && idleCoilCount > 0;
    if (needMoving){
        curState = CALCULATING;
    }

    return 0;
}

int scheduleCalculating(){
    if (toSchedule.size() > 0 && idelCoilCount == 0) {
        curState = WAITING;
        return 0;
    }
    
    // Schedule the charging of devices if there are any

    // Collect ChargerManager::CHARGER_COUNT devices and reschedule all of them

    // Move the coils to corner if there are idel ones
    
    return 0;
}

int scheduleMoving1(){
    return 0;
}

int scheduleMoving2(){
    return 0;
}

int scheduleError(){
    printf("Error: %s\n", error_message.c_str());
    return 0;
}

