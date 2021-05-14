
// for delay function.
#include <chrono>
#include <thread>

#include "Control.h"
#include "Vision.h"
#include "ChargerManager.h"
#include "GrabberController.h"

// FIXME: [liuzikai] control thread should be robust for any possible errors
#define ERROR_(message_) { \
    errorMessage = message_; \
    curState = ERROR; \
    return -1; }

using namespace std;


/********************************* Public Functions *****************************/

Control::Control(Vision *vision, ChargerManager *chargerManager, GrabberController *grabberController)
        : vision(vision), chargerManager(chargerManager), grabberController(grabberController) {

    curState = WAITING;
}

int Control::launch() {
    // Control thread never exits
    while (true) {
        std::cout << "current state: " << curState << std::endl;
        switch (curState) {
            case WAITING:
                scheduleWaiting();
                break;
            case CALCULATING:
                scheduleCalculating();
                break;
            case MOVING1:
                scheduleMoving1();
                break;
            case MOVING2:
                scheduleMoving2();
                break;
            case ERROR:
                scheduleError();
                break;
            default:
                assert(!"Invalid state");
        }

        if (curState == WAITING) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            vision->setAcceptImages(true);
        } else {
            vision->setAcceptImages(false);
        }
    }
}


/********************************* Scheduling Controls *****************************/

int Control::scheduleWaiting() {

    bool needMoving = false;
    set<cv::Point, PointLess> toIgnore;   // filled when new status is CHARGING
    // set<cv::Point, PointLess> toConfirm;  // filled when new status is NOT_CHARGING

    // Pull for the wireless charging status
    for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++) {
        auto curStatus = chargerManager->getChargerStatus(i);

        // Retry for once if the status is unknown
        // FIXME: [liuzikai] retry for more times
        if (curStatus == ChargerManager::UNKNOWN) {
            usleep(1000000);
            curStatus = chargerManager->getChargerStatus(i);

            if (curStatus == ChargerManager::UNKNOWN) {
                ERROR_("Pull charging status failed");
            }
        }

        // Handle if there is a new status
        if (curStatus != oldStatus[i]) {
            oldStatus[i] = curStatus;

            switch (curStatus) {

                // A device is put at the coil
                case ChargerManager::CHARGING:
                    // Let it go and update available coils

                    // The new device position at this coil should be ignored later
                    if (chargeable.find(curCoilPositions[i]) != chargeable.end()) {
                        ERROR_("Duplicate Device!");
                    }
                    toIgnore.insert(curCoilPositions[i]);
                    chargeable.emplace(curCoilPositions[i], Device(curCoilPositions[i]));

                    if (unchargeable.find(curCoilPositions[i]) != unchargeable.end()) {
                        unchargeable.erase(curCoilPositions[i]);
                    }
                    break;

                    // A device is removed/finished charging
                case ChargerManager::NOT_CHARGING: {
                    
                    
                    auto it = chargeable.begin();
                    for (; it != chargeable.end(); it++){
                        if (it->second.coor == curCoilPositions[i]) break;
                    }

                    if (it != chargeable.end()) {
                        unchargeable.emplace(*it); // would be handled if the device is removed
                        chargeable.erase(it->first);
                    } else {
                        cerr << "Coil is at " << curCoilPositions[i].x << "," << curCoilPositions[i].y  << endl;
                        cerr << "Chargeable devices:" << endl;
                        for (const auto &charge : chargeable){
                            cerr << charge.first.x << "," << charge.first.y << " -> " <<
                            charge.second.coor.x << "," << charge.second.coor.y << endl;
                        }
                        ERROR_("A original chargeing device is not in chargeable!");
                    }

                    // toConfirm.insert(curCoilPositions[i]);
                    needMoving = true;
                }
                    break;
                default:
                    assert(0);
            }
        }
    }

    // Pull for vision updates
    if (idleCoilCount() > 0){
        vector<cv::RotatedRect> newDevices, removedDevices;
        vision->fetchDeviceDiff(newDevices, removedDevices);

        for (const auto &newDevice : newDevices) {
            // Check whether this new device should be ignored
            if (toIgnore.find(newDevice.center) != toIgnore.end()) {
                toIgnore.erase(newDevice.center);
                continue;
            }

            toSchedule.insert(newDevice);
        }

        for (const auto &removedDevice : removedDevices) {

            // Check the confirm: the removal has already been noticed by wireless charging
            // FIXME: [liuzikai] coordination from curCoilPositions and from vision can hardly match exactly, use range compare

            // if (toConfirm.find(removedDevice.center) != toConfirm.end()) {

            //     toConfirm.erase(removedDevice.center);

            //     // The device was charging but taken away
            //     // if (!chargeable.erase(removedDevice)) {
            //     //     ERROR_("Chargeable map panic");
            //     // }

            //     continue;
            // }

            // Remove the device that is either unchargeable or not scheduled
            if (!unchargeable.erase(removedDevice.center)) {
                if (toSchedule.find(removedDevice) != toSchedule.end()) {
                    toSchedule.erase(removedDevice);
    //                    ERROR_("Unchargeable map or to schedule panic");
                }
            }
        }


        // if (toIgnore.size() != 0){
        //     ERROR_("Conflict message from wireless and vision!");
        // }

        // Mark the remaining toConfirm devices as unchargeable (finished charging)
        // for (const auto &confirm : toConfirm) {
        //     auto curDevice = chargeable.find(confirm);

        //     if (curDevice != chargeable.end()) {
        //         unchargeable.emplace(*curDevice);
        //         chargeable.erase(curDevice->first);
        //     } else {
        //         ERROR_("Conflict message from wireless and vision!");
        //     }
        // }
    }
    

    needMoving |= !toSchedule.empty() && idleCoilCount() > 0;
    if (needMoving) {
        curState = CALCULATING;
    }

    return 0;
}

int Control::scheduleCalculating() {
    if (!toSchedule.empty() && idleCoilCount() == 0) {
        curState = WAITING;
        return 0;
    }

    // Schedule the charging of devices if there are any
    // Collect ChargerManager::CHARGER_COUNT devices and reschedule all of them
    set<cv::Point, PointLess> curSchedule;  // device to schedule
    map<cv::Point, cv::RotatedRect, PointLess> newDeviceMapping;
    int availableCoils = idleCoilCount();
    while (!toSchedule.empty() && availableCoils > 0) {
        auto curDevice = *toSchedule.begin();
        curSchedule.insert(curDevice.center);
        newDeviceMapping.insert(make_pair(curDevice.center, curDevice));
//        schedulingNew.insert(curDevice.center);
        toSchedule.erase(curDevice);
        availableCoils--;
    }

    // New device to schedule
    unordered_map<int, cv::Point> coilTarget;
    if (!curSchedule.empty()) {

        // Collect currently charging devices into the scheduling
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++) {
            if (oldStatus[i] == ChargerManager::CHARGING) {
                curSchedule.insert(curCoilPositions[i]);
//                schedulingOld.insert(curCoilPositions[i]);
            }
        }

        // Assign the coils according to the device location
        // Method: [assign the coil to the device] that have the cloest distance its initial position
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++) {

            // Iterate through to find the target point that is closest to the coil's initial position
            cv::Point targetPoint;
            double minDist = INFINITY;
            for (const auto &point : curSchedule) {
                double dist = cv::norm(initialPositions[i] - point);
                if (dist < minDist) {
                    targetPoint = point;
                    minDist = dist;
                }
            }

            // Add the device with the smallest distance to be the target
            if (targetPoint != curCoilPositions[i]) coilTarget.insert(make_pair(i, targetPoint));
            curSchedule.erase(targetPoint);
        }

        // Fill the moving queue TODO: also change the order of the coilTarget map
        // Order: to avoid coil conflict, the devices with no coil under it are scheduled first
        // Start with a new devices (garentee to have no coil under it)
        // for (const auto& newSche : schedulingNew){
        //     auto curTarget = coilTarget[newSche];
        //     movingCommands.push(make_pair(curTarget.second, curTarget.first));

        // }


        for (const auto &target : coilTarget) {
            if (newDeviceMapping.find(target.second) != newDeviceMapping.end()) {
                movingCommands.push(make_pair(target.first, newDeviceMapping[target.second]));
            } else {
                movingOldCommands.push(make_pair(target.first, target.second));
            }
        }
    }

    // Move the coils to corner if there are idle ones 
    for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++) {

        if (oldStatus[i] == ChargerManager::NOT_CHARGING &&
            coilTarget.find(i) == coilTarget.end() &&
            curCoilPositions[i] != initialPositions[i]) {

            movingIdleCommands.push(make_pair(i, initialPositions[i]));
        }
    }

    if (movingCommands.empty() && movingOldCommands.empty() && movingIdleCommands.empty() ) curState = WAITING;
    else curState = MOVING1;

    return 0;
}

int Control::scheduleMoving1() {

    bool needMoving = false; // for moving the idle coils to initial position

    // Move the idle coils to initial first
    while (!movingIdleCommands.empty()) {
        auto c = movingIdleCommands.front();
        movingIdleCommands.pop();
        auto &coil = curCoilPositions[c.first];
        grabberController->moveGrabber(coil.x, coil.y, GrabberController::SPEED_FAST);
        grabberController->moveGrabber(c.second.x, c.second.y);
        grabberController->detachGrabber();
        curCoilPositions[c.first] = {c.second.x, c.second.y};
        sleep(4);  // TODO: guarantee to finish!
    }

    // Send the moving commands for new devices
    while (!movingCommands.empty()) {

        // Issue the command
        auto c = movingCommands.front();
        movingCommands.pop();
        auto &curDevice = c.second;
        auto &coil = curCoilPositions[c.first];

        grabberController->moveGrabber(coil.x, coil.y, GrabberController::SPEED_FAST);

        float finalX = curDevice.center.x;
        float finalY = curDevice.center.y;

        grabberController->moveGrabber(finalX, finalY);

        // Wait until complete and check the final wireless charging status
        sleep(7); // TODO: guarantee to finish!

        // Update the status according to wireless read

        auto curStatus = chargerManager->getChargerStatus(c.first);

        if (curStatus != ChargerManager::CHARGING) {

            // Explore
            float longEdge, shortEdge;
            float angle;  // angle from X to shortEdge
            if (curDevice.size.width < curDevice.size.height) {
                shortEdge = curDevice.size.width;
                longEdge = curDevice.size.height;
                angle = curDevice.angle + 180;
            } else {
                shortEdge = curDevice.size.height;
                longEdge = curDevice.size.width;
                angle = curDevice.angle + 90;
            }

            // The offset for each explore
            float shortEdgeStep[2] = {  // (x, y)
                    static_cast<float>(shortEdge * cos(angle * M_PI / 180.0f)),
                    static_cast<float>(shortEdge * -sin(angle * M_PI / 180.0f))
            };
            // std::cerr << "shortEdgeStep: " << shortEdgeStep[0] << ", " << shortEdgeStep[1] << std::endl;
            float longEdgeStep[2] = {  // (x, y)
                    static_cast<float>(longEdge * sin(angle * M_PI / 180.0f)),
                    static_cast<float>(longEdge * -cos(angle * M_PI / 180.0f))
            };
            // std::cerr << "longEdgeStep: " << longEdgeStep[0] << ", " << longEdgeStep[1] << std::endl;

            static const float explorePath[][2] = {  // (long edge, short edge)
                {0, 0}, {0.1, 0}, {0.2, 0}, {0.3, 0}, {0.3, 0}, {0.2, 0}, {0.1, 0}, {0, 0},
                {0, 0}, {-0.1, 0}, {-0.2, 0}, {-0.3, 0}, {-0.3, 0}, {-0.2, 0}, {-0.1, 0}, {0, 0},
                {0, 0}, {0, 0.15}, {0, 0.3}, {0, 0.4}, {0, 0.4}, {0, 0.2}, {0, 0.1}, {0, 0},
                {0, 0}, {0, -0.15}, {0, -0.3}, {0, -0.4}, {0, -0.4}, {0, -0.3}, {0, -0.15}, {0, 0}
            };

            int p;
            for (p = 0; p < sizeof(explorePath) / sizeof(explorePath[0]); p++) {
                finalX = curDevice.center.x + explorePath[p][1] * shortEdgeStep[0] + explorePath[p][0] * longEdgeStep[0];
                finalY = curDevice.center.y + explorePath[p][1] * shortEdgeStep[1] + explorePath[p][0] * longEdgeStep[1];
                grabberController->moveGrabber(finalX, finalY, GrabberController::SPEED_SLOW);
                std::this_thread::sleep_for(std::chrono::milliseconds(750));
                curStatus = chargerManager->getChargerStatus(c.first);
                if (curStatus == ChargerManager::CHARGING) break;
            }

            if (curStatus == ChargerManager::CHARGING && p > 0) {
                p--;  // move back for one step
                finalX = curDevice.center.x + explorePath[p][1] * shortEdgeStep[0] + explorePath[p][0] * longEdgeStep[0];
                finalY = curDevice.center.y + explorePath[p][1] * shortEdgeStep[1] + explorePath[p][0] * longEdgeStep[1];
                grabberController->moveGrabber(finalX, finalY, GrabberController::SPEED_SLOW);
            }
        }

        grabberController->detachGrabber();

        // 4 cases for wireless coil status change {Charging, Not charging} -> {Charging, Not charging}
        // According to the notes, they can be merged. Only the final status matter

        if (curStatus == ChargerManager::CHARGING) {

            // Add the device if it is a new chargebale device
            if (chargeable.find(curDevice.center) == chargeable.end()) {
                chargeable.emplace(curDevice.center, Device(cv::Point(finalX, finalY)));
            }

            if (unchargeable.find(curDevice.center) != unchargeable.end()) {
                unchargeable.erase(curDevice.center);
            }


        } else {

            needMoving = true;

            if (unchargeable.find(curDevice.center) == unchargeable.end()) {
                unchargeable.emplace(curDevice.center, Device(cv::Point(finalX, finalY)));
            }

            if (chargeable.find(curDevice.center) != chargeable.end()) {
                chargeable.erase(curDevice.center);
            }
        }

        oldStatus[c.first] = curStatus;
        curCoilPositions[c.first] = cv::Point(finalX, finalY);

    }

    // Move for the rescheduled devices
    while (!movingOldCommands.empty()) {
        auto c = movingOldCommands.front();
        movingOldCommands.pop();

        // Check for collision
        bool collision = false;
        for (int i = 0; i < ChargerManager::CHARGER_COUNT; i++){
            auto &coil = curCoilPositions[i];
            if (coil.x == c.second.x && coil.y == c.second.y){
                collision = true;
                movingOldCommands.push(c);
                break;
            }
        }
        if (collision) continue; 

        auto &coil = curCoilPositions[c.first];
        grabberController->moveGrabber(coil.x, coil.y, GrabberController::SPEED_FAST);
        grabberController->moveGrabber(c.second.x, c.second.y);
        grabberController->detachGrabber();
        curCoilPositions[c.first] = {c.second.x, c.second.y};

        sleep(10); // TODO: guarantee to finish!

        auto curStatus = chargerManager->getChargerStatus(c.first);

        if (curStatus != ChargerManager::CHARGING) {

            // Rescheduled device no longer chargeable
            auto re = chargeable.find(c.second);

            if (re != chargeable.end()) {
                unchargeable.emplace(*re);
                chargeable.erase(re->first);
            } else {
                ERROR_("Chargeable map panic at reschedule");
            }

            needMoving = true;

        }

        oldStatus[c.first] = curStatus;
    }


    if (needMoving) {
        curState = CALCULATING;
    } else {
        grabberController->resetGrabber();
        sleep(5);
        curState = WAITING;
    }

    return 0;
}

int Control::scheduleMoving2() {
    return 0;
}

int Control::scheduleError() {
    sleep(5);
    printf("Error: %s\n", errorMessage.c_str());
    return 0;
}

