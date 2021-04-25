#ifndef CONTROL_H
#define CONTROL_H

// Created by Tingkai Liu 2021-03-29

#include <vector>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>
#include <thread>

#include "ChargerManager.h"
#include "GrabberController.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

class Control {
public:

    Control();

    // ~Control();

    bool controlMoving() {return curState == MOVING1 || curState == MOVING2; };

private:

    enum State {
        WAITING, CALCULATING, MOVING1, MOVING2, ERROR, NUM_STATES
    };

    std::thread *th = nullptr;

    int launch();  // thread body, never returns

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

    struct PointHash {
        size_t operator()(cv::Point const &a) const {
            return a.x * 500 + a.y;  // random choice
        }
    };

    // For the using of unordered_map on cv::RotatedRect TODO: stricter?
    struct RotatedRectEqual {
        bool operator()(cv::RotatedRect const &a, cv::RotatedRect const &b) const {
            return (a.center.x == b.center.x) && (a.center.y == b.center.y);
        }
    };

    struct PointEqual {
        bool operator()(cv::Point const &a, cv::Point const &b) const {
            return (a.center.x == b.center.x) && (a.center.y == b.center.y);
        }
    };

    struct RotatedRectUnequal {
        bool operator()(cv::RotatedRect const &a, cv::RotatedRect const &b) const {
            return (a.center.x != b.center.x) || (a.center.y != b.center.y);
        }
    };

    bool operator!=(cv::Point const &a, cv::Point const &b) const {
        return (a.x != b.x) || (a.y != b.y);
    }

    bool operator==(cv::Point const &a, cv::Point const &b) const {
        return (a.x == b.x) && (a.y == b.y);
    }

    struct Device {
        Device(cv::RotatedRect coor) : coor(coor) {}


        cv::RotatedRect coor;

        // The x, y dimention length of the device
        // int x_length;
        // int y_length;

        bool chargable;
    };

    /********************************* Scheduling Controls *****************************/

    using ScheduleFunction = int (*)();

    ScheduleFunction schedule[NUM_STATES];

    /**
     * The scheduling functions for each states
     * See the document for the work of each state
     */
    int scheduleWaiting();

    int scheduleCalculating();

    int scheduleMoving1();

    int scheduleMoving2();

    int scheduleError();


    ChargerManager chargerManager;
    GrabberController grabberController;

    /********************************* Data Fields *****************************/

    State curState;

    std::string errorMessage;

    // Device status
    std::unordered_map<cv::Point, Device, PointHash, PointEqual> chargeable;
    std::unordered_map<cv::Point, Device, PointHash, PointEqual> unchargeable;

    std::set<cv::RotatedRect, RotatedRectLess> toSchedule;

    std::set<cv::RotatedRect, RotatedRectLess> schedulingNew;  // the new devices in scheduling
    std::set<cv::RotatedRect, RotatedRectLess> schedulingOld;  // the old devices rescheduling (coil change)

    std::queue<std::pair<int, cv::RotatedRect> > movingCommands;  // (coil index, target)
    std::queue<std::pair<int, cv::Point> > movingOldCommands; // (coil index, target) for rescheduled old devices
    std::queue<std::pair<int, cv::Point> > movingIdleCommands; // (coil index, target) for moving idle coils to initial position

    cv::Point initialPositions[ChargerManager::CHARGER_COUNT] = {cv::Point(0, 0)};
    cv::Point curCoilPositions[ChargerManager::CHARGER_COUNT] = {cv::Point(0, 0)};

    // For wireless charging subsystem
    int idleCoilCount;
    ChargerManager::Status oldStatus[ChargerManager::CHARGER_COUNT] = {ChargerManager::NOT_CHARGING};


};


#endif /* #ifndef CONTROL_H */
