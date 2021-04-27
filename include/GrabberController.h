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
     * @param x     The designation X coordinate
     * @param y     The designation Y coordinate
     * @param fast  Whether to move the grabber in the fast speed
     */
    void moveGrabber(float x, float y, bool fast = false);

    /**
     * Detach the grabber and move back to (0, 0)
     */
    void resetGrabber();

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

    static constexpr unsigned MOVE_SPEED_DETACHED = 42000;  // [mm/min]
    static constexpr unsigned MOVE_SPEED_ATTACHED = 12000;  // [mm/min]
};


#endif //QI_TABLE_MEINTERFACE_H
