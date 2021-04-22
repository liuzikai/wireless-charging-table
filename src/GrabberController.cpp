//
// Created by liuzikai on 4/13/21.
//

#include "GrabberController.h"
#include <iomanip>

GrabberController::GrabberController(const string &serialDevice, unsigned baudRate)
        : serial(ioContext, serialDevice) {

    serial.set_option(boost::asio::serial_port::baud_rate(baudRate));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

    serialSendCommand("G90");  // set absolute positioning
}

void GrabberController::issueGrabberMovement(float srcX, float srcY, float destX, float destY) {
    // Step 1: attach the grabber from src
    issueMoveCommand(MOVE_SPEED_DETACHED, srcX, srcY, Z_ATTACHED);

    // Step 2: move the coil
    issueMoveCommand(MOVE_SPEED_ATTACHED, destX, destY);

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

void GrabberController::serialSendCommand(const string &s) {
    boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
    boost::asio::write(serial, boost::asio::buffer("\n", 1));

    /*boost::asio::streambuf buf;
    boost::system::error_code ec;
    auto readLen = boost::asio::read_until(serial, buf, '\n', ec);
    if (ec) {
        std::cerr << "\"" << s << "\" -> fails to read back" << std::endl;
    } else {
        std::string info{buffers_begin(buf.data()), buffers_begin(buf.data()) + readLen - 1};
        if (info != "ok") {
            std::cerr << "\"" << s << "\" ->  \"" << info << "\"" << std::endl;
        }
    }*/
}
