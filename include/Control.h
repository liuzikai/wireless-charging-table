#ifndef CONTROL_H
#define CONTROL_H

// Created by Tingkai Liu 2021-03-29

#include "Common.h"
#include "ChargerManager.h"
#include <utility>
#include <vector>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>
#include <thread>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

// Forward declaration
class Vision;
class ChargerManager;
class GrabberController;

class Control {
public:

    explicit Control(Vision *vision, ChargerManager *chargerManager, GrabberController *grabberController);

    int launch();  // thread body, never returns

private:

    Vision *vision;
    ChargerManager *chargerManager;
    GrabberController *grabberController;

    enum State {
        WAITING, CALCULATING, MOVING1, MOVING2, ERROR, NUM_STATES
    };

    struct RotatedRectLess {
        bool operator()(cv::RotatedRect const &a, cv::RotatedRect const &b) const {
            if (a.center.x == b.center.x) return a.center.y < b.center.y;
            else return a.center.x < b.center.x;
        }
    };

    struct PointLess {
        bool operator()(cv::Point const &a, cv::Point const &b) const {
            if (a.x == b.x) return a.y < b.y;
            else return a.x < b.x;
        }
    };

    struct Device {
        Device(cv::Point coor) : coor(std::move(coor)) {}


        cv::Point coor;

        // The x, y dimention length of the device
        // int x_length;
        // int y_length;

        bool chargable;
    };

    /********************************* Scheduling Controls *****************************/


    /**
     * The scheduling functions for each states
     * See the document for the work of each state
     */
    int scheduleWaiting();

    int scheduleCalculating();

    int scheduleMoving1();

    int scheduleMoving2();

    int scheduleError();


    /********************************* Data Fields *****************************/

    State curState;

    std::string errorMessage;

    // Device status
    std::map<cv::Point, Device, PointLess> chargeable;
    std::map<cv::Point, Device, PointLess> unchargeable;

    std::set<cv::RotatedRect, RotatedRectLess> toSchedule;

//    std::set<cv::RotatedRect, RotatedRectLess> schedulingNew;  // the new devices in scheduling
//    std::set<cv::RotatedRect, RotatedRectLess> schedulingOld;  // the old devices rescheduling (coil change)

    std::queue<std::pair<int, cv::RotatedRect> > movingCommands;  // (coil index, target)
    std::queue<std::pair<int, cv::Point> > movingOldCommands; // (coil index, target) for rescheduled old devices
    std::queue<std::pair<int, cv::Point> > movingIdleCommands; // (coil index, target) for moving idle coils to initial position

    cv::Point initialPositions[ChargerManager::CHARGER_COUNT] = {cv::Point(0, 0), cv::Point(340, 0), cv::Point(0, 350)};
    cv::Point curCoilPositions[ChargerManager::CHARGER_COUNT] = {cv::Point(0, 0), cv::Point(340, 0), cv::Point(0, 350)};

    // For wireless charging subsystem
    ChargerManager::Status oldStatus[ChargerManager::CHARGER_COUNT] = {ChargerManager::NOT_CHARGING, ChargerManager::NOT_CHARGING, ChargerManager::NOT_CHARGING};
    int idleCoilCount() const {
        int ret = 0;
        for (const auto &status : oldStatus) if (status == ChargerManager::NOT_CHARGING) ret++;
        return ret;
    };


};


#endif /* #ifndef CONTROL_H */
