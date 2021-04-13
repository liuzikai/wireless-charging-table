//
// Created by liuzikai on 4/13/21.
//

#include "GrabberController.h"
#include <iomanip>

GrabberController::GrabberController(const string &serialDevice, unsigned baudRate)
        : serial(ioContext, serialDevice) {

    serial.set_option(boost::asio::serial_port::baud_rate(baudRate));

    serialSendCommand("G90");  // set absolute positioning
}

void GrabberController::issueGrabberMovement(float srcX, float srcY, float destX, float destY) {
    // Step 1: attach the grabber from src
    issueMoveCommand(MOVE_SPEED_DETACHED, srcX, srcY, Z_ATTACHED);

    // Pause for a while for the grabber to attach
    issueDwellCommand(DWELL_TIME_MS);

    // Step 2: move the coil
    issueMoveCommand(MOVE_SPEED_ATTACHED, destX, destY);

    // Pause for a while
    issueDwellCommand(DWELL_TIME_MS);

    // Step 3: release the coil
    issueMoveCommand(MOVE_SPEED_DETACHED, -1, -1, Z_DETACHED);
}

void GrabberController::issueMoveCommand(unsigned speed, float x, float y, float z) {
    stringstream ss;

    ss << std::fixed << std::setprecision(FLOAT_PRECISION);

    // Use G0 for non-print moves
    ss << "G0 F" << speed;
    if (x != -1) ss << " X" << x;
    if (y != -1) ss << " Y" << y;
    if (z != -1) ss << " Z" << z;

    serialSendCommand(ss.str());
}

void GrabberController::issueDwellCommand(int ms) {
    serialSendCommand("G4 P" + std::to_string(ms));
}

void GrabberController::serialSendCommand(const string &s) {
    boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
    // TODO: confirm the delimiter of commands
    boost::asio::write(serial, boost::asio::buffer("\n"));
}
