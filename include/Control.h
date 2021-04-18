#ifndef CONTROL_H
#define CONTROL_H

// Created by Tingkai Liu 2021-03-29

#include <vector>
#include <unordered_map>
#include <string>
#include <set>
#include <queue>

#include "ChargerManager.h"
#include "GrabberController.h"
#include "Vision.h"

// For the using of unorder_map on cv::Point
bool operator==(cv::Point const& a, cv::Point const& b){
    return (a.x == b.x) && (a.y == b.y);
}

bool operator<(cv::Point const& a, cv::Point const& b){
    if (a.x == b.x) return a.y < b.y;
    else return a.x < b.x;
}

class MyHash{
    size_t operator()(cv::Point const& a) const {
        return a.x * 500 + a.y; // random choice
    }
};

struct Device{
    Device(cv::Point coor) : coor(coor) {}
    
    
    cv::Point coor;

    // The x, y dimention length of the device
    // int x_length;
    // int y_length;

    bool chargable;
};

typedef int (*ScheduleFunction)(void);

class Control {

public:
    Control();

    // ~Control();

    enum State{
        WAITING, CALCULATING, MOVING1, MOVING2, ERROR, NUM_STATES
    };

    /**
     * Launch the all the controls
     * The functions goes into an infinite loop of controlling and never returns
     */
    int launch();


private:
    /********************************* Schduling Controls *****************************/
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
    std::unordered_map<cv::Point, Device, MyHash> chargeable;
    std::unordered_map<cv::Point, Device, MyHash> unchargeable;
    
    std::set<cv::Point> toSchedule;
    std::queue<std::pair<cv::Point, cv::Point> > movingCommands;

    cv::Point curCoilPositions[ChargerManager::CHARGER_COUNT];

    // For Wireless Charging Subsystem
    int idleCoilCount;
    ChargerManager::Status oldStatus[ChargerManager::CHARGER_COUNT];


};


#endif /* #ifndef CONTROL_H */
