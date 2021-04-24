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
     * @param serialDevice
     * @param baudRate
     */
    explicit GrabberController(const string &serialDevice, unsigned baudRate);

    /**
     * Issue a series of movements to move a coil: attach, move, detach.
     * @param srcX   The current X coordinate of the coil
     * @param srcY   The current Y coordinate of the coil
     * @param destX  The designation X coordinate
     * @param destY  The designation Y coordinate
     */
    void issueGrabberMovement(float srcX, float srcY, float destX, float destY);

private:

    void issueMoveCommand(unsigned speed, float x = -1, float y = -1, float z = -1);

    boost::asio::io_context ioContext;
    boost::asio::serial_port serial;

    void serialSendCommand(const string &s);

    static constexpr unsigned FLOAT_PRECISION = 1;

    // TODO: calibrate on the mechanical structure
    static constexpr float Z_DETACHED = 0;
    static constexpr float Z_ATTACHED = 50;

    static constexpr unsigned MOVE_SPEED_DETACHED = 42000;  // [mm/min]
    static constexpr unsigned MOVE_SPEED_ATTACHED = 12000;  // [mm/min]
};


#endif //QI_TABLE_MEINTERFACE_H
