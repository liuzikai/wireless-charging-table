
// for delay function.
#include <chrono> 
#include <thread>

#include <stdio.h>
#include <unistd.h>
// #include <pthread.h>
#include <assert.h>

#include <JetsonGPIO.h>
#include <set>
#include <map>
#include <algorithm>

#include "../include/Control.h"

#define ERROR_(message_) { \
    errorMessage = message_; \
    curState = ERROR; \
    return -1; }


using namespace std;


// TODO: Get from vision
static volatile bool visionNeedsHandling;
static volatile vector<cv::Point> newDevices;
static volatile vector<cv::Point> removedDevices;

// TODO: Move to charger manager
static const vector<cv::Point> initialPositions;

// static pthread_mutex_t pin_locks[ChargerManager::CHARGER_COUNT]; 


/********************************* Public Functions *****************************/

Control::Control() : chargerManager(), grabberController() {
    
    curState = WAITING;

    // Fill the schedule functions table
    schedule[0] = &Control::scheduleWaiting;
    schedule[1] = &Control::scheduleCalculating;
    schedule[2] = &Control::scheduleMoving1;
    schedule[3] = &Control::scheduleMoving2;
    schedule[4] = &Control::scheduleError;

    // Init the coil positions TODO: set default value at .h
    // curCoilPositions[0] = cv::Point(0,0);
    // curCoilPositions[1] = cv::Point(0,0);
    // curCoilPositions[2] = cv::Point(0,0);
    idleCoilCount = ChargerManager::CHARGER_COUNT;


}

int Control::launch(){
    
    while(1){
        schedule[curState]();
        sleep(1); // Sleep for 1s
    }

    return 0; // Should never reach here
}


/********************************* Schduling Controls *****************************/

int Control::scheduleWaiting(){
    
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

            if (curStatus == ChargerManager::UNKNOWN){
                ERROR_("Pull charging status failed");
            }
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
                    if (chargeable.find(curCoilPositions[i]) != chargeable.end()){
                        ERROR_("Duplicate Device!");
                    }
                    toIgnore.insert(curCoilPositions[i]);
                    chargeable.insert(make_pair(curCoilPositions[i], Device(curCoilPositions[i])));

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

            toSchedule.insert(newDevice);

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

        visionNeedsHandling = false;
        newDevices.clear();
        removedDevices.clear();

    }

    // if (toIgnore.size() != 0){
    //     ERROR_("Conflict message from wireless and vision!");
    // }

    // Mark the remaining toConfirm devices as unchargeable (finished charging)
    for (const auto& confirm : toConfirm){
        auto curDevice = chargeable.find(confirm);
        
        if (curDevice != chargeable.end()){
            unchargeable.emplace(curDevice);
            chargeable.erase(curDevice->first);
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

int Control::scheduleCalculating(){
    if (toSchedule.size() > 0 && idleCoilCount == 0) {
        curState = WAITING;
        return 0;
    }
    
    // Schedule the charging of devices if there are any
    // Collect ChargerManager::CHARGER_COUNT devices and reschedule all of them
    set<cv::Point> curSchedule;
    int availableCoils = idleCoilCount;
    while(toSchedule.size() > 0 && availableCoils > 0){
        auto curDevice = *toSchedule.begin();
        curSchedule.insert(curDevice);
        schedulingNew.insert(curDevice);
        toSchedule.erase(curDevice);
        availableCoils--;
    }

    // New device to schedule
    unordered_map<cv::Point, int> coilTarget;
    if (curSchedule.size() > 0){
        
        // Collect currently charging devices into the scheduling
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
            if (oldStatus[i] == ChargerManager::CHARGING){
                curSchedule.insert(curCoilPositions[i]);
                schedulingOld.insert(curCoilPositions[i]);
            }
        }

        // Assign the coils according to the device location
        // Method: [assign the coil to the device] that have the cloest distance its initial position
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
            
            // Didn't use map here since the distance may be the same for different devices
            vector<pair<double, cv::Point> > distance; 
            
            // Collect the distances
            for (const auto& device : curSchedule){
                distance.push_back(make_pair(cv::norm(initialPositions[i] - device), device));
            }

            // Sort the distances
            sort(distance.begin(), distance.end(), 
                [] (const pair<double, cv::Point>& a, const pair<double, cv::Point>& b) -> bool {
                    return a.first < b.first;
                });
            
            // Add the device with the smallest distance to be the target
            cv::Point curTarget = distance[0].second;
            curSchedule.erase(curTarget);
            if (curTarget != curCoilPositions[i]) coilTarget.insert(make_pair(curTarget, i));
            
        }
        
        // Fill the moving queue TODO:
        // Order: to avoid coil conflict, the devices with no coil under it are scheduled first
        // Start with a new devices (garentee to have no coil under it)
        // for (const auto& newSche : schedulingNew){
        //     auto curTarget = coilTarget[newSche];
        //     movingCommands.push_back(make_pair(curTarget.second, curTarget.first));

        // }
        
        
        for (const auto& target : coilTarget){
            movingCommands.push_back(make_pair(target.first, target.second));
        }
    }

    // Move the coils to corner if there are idle ones 
    for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
        
        if (oldStatus[i] == ChargerManager::NOT_CHARGING && 
            coilTarget.find(i) == coilTarget.end() &&
            curCoilPositions[i] != initialPositions[i]){
            
            movingCommands.push(make_pair(i, initialPositions[i]));
        
        }
    }

    
    if(movingCommands.size() > 0) curState = MOVING1;
    else curState = WAITING;

    return 0;
}

int Control::scheduleMoving1(){
    // Send the moving commands
    while(!movingCommands.empty()){
        
        // Issue the command
        auto c = movingCommands.front();
        movingCommands.pop();
        auto& coil = curCoilPositions[c.first]
        grabberController.issueGrabberMovement(coil.x, coil.y, c.second.x, c.second.y);

        // Wait until complete and check the final wireless charging status
        sleep(5); // TODO: garenttee to finish!

        // Update the status according to wireless read

        auto curStatus = chargerManager.getChargerStatus(c.first);
        // Retry for once if the status is unknown
        if (curStatus == ChargerManager::UNKNOWN) {
            usleep(500000);
            curStatus = chargerManager.getChargerStatus(c.first);

            if (curStatus == ChargerManager::UNKNOWN){
                ERROR_("Pull charging status failed");
            }
        }

        // 4 cases for wireless coil status change {Charging, Not charging} -> {Charging, Not charging}
        // According to the notes, they can be merged. Only the final status matter
        
        if (curStatus == ChargerManager::CHARGING){
             
            // Add the device if it is a new chargebale device
            if (chargeable.find(c.second) == chargeable.end()){
                
                chargeable.insert(make_pair(c.second, Device(c.second)));  

            }


        } else {
            
            if (unchargeable.find(c.second) == unchargeable.end()){
                
                unchargeable.insert(make_pair(c.second, Device(c.second)));  

            }
        }

        oldStatus[c.first] = curStatus;

    }
    

    curState = WAITING;
    return 0;
}

int Control::scheduleMoving2(){
    return 0;
}

int Control::scheduleError(){
    printf("Error: %s\n", errorMessage.c_str());
    return 0;
}

