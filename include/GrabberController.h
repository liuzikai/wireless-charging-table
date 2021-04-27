//
// Created by liuzikai on 4/13/21.
//

#ifndef QI_TABLE_MEINTERFACE_H
#define QI_TABLE_MEINTERFACE_H

#include "Common.h"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class GrabberController {
public:

    /**
     * Instantiate a GrabberController using the given serial port.
     */
    explicit GrabberController();

    /**
     * Move the grabber to the target position, attached. This function returns immediately.
     * @param x      The designation X coordinate
     * @param y      The designation Y coordinate
     * @param speed  Moving speed
     */
    void moveGrabber(float x, float y, unsigned speed = SPEED_NORMAL);

    /**
     * Detach grabber
     */
    void detachGrabber();

    /**
     * Detach the grabber and move back to (0, 0)
     */
    void resetGrabber();

    static constexpr unsigned SPEED_FAST = 42000;    // [mm/min]
    static constexpr unsigned SPEED_NORMAL = 12000;  // [mm/min]
    static constexpr unsigned SPEED_SLOW = 600;      // [mm/min]

private:

    void issueMoveCommand(unsigned speed, float x = -1, float y = -1, float z = -1);

    boost::asio::io_context ioContext;
    boost::asio::serial_port serial;

    static constexpr const char *SERIAL_DEVICE_PREFIX = "/dev/ttyACM";
    static constexpr int SERIAL_BAUD_RATE = 115200;

    void serialSendCommand(const string &s);

    static constexpr unsigned FLOAT_PRECISION = 1;

    static constexpr float Z_DETACHED = 30;
    static constexpr float Z_ATTACHED = 50;
};


#endif //QI_TABLE_MEINTERFACE_H
