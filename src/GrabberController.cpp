//
// Created by liuzikai on 4/13/21.
//

#include "GrabberController.h"
#include <iomanip>

GrabberController::GrabberController()
        : serial(ioContext) {

    // Enumerate /dev/ttyACM0, /dev/ttyACM1, ..., /dev/ttyACM9
    boost::system::error_code ec;
    for (int i = 0; i < 10; i++) {
        string device = SERIAL_DEVICE_PREFIX + std::to_string(i);
        std::cout << "Serial: " << device << "...";
        serial.open(device, ec);
        if (!ec) {
            std::cout << "succeeded" << std::endl;
            break;
        } else {
            std::cout << "failed " << ec.message() << std::endl;
        }
    }
    assert(!ec && "Failed to open the serial device");

    serial.set_option(boost::asio::serial_port::baud_rate(SERIAL_BAUD_RATE));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

    serialSendCommand("G28");  // home
}

void GrabberController::moveGrabber(float x, float y, unsigned speed) {
    issueMoveCommand(speed, x, y, Z_ATTACHED);
}

void GrabberController::detachGrabber() {
    issueMoveCommand(SPEED_FAST, -1, -1, Z_DETACHED);
}

void GrabberController::resetGrabber() {
    // Step 1: detach
    issueMoveCommand(SPEED_FAST, -1, -1, Z_DETACHED);

    // Step 2: move back to the origin
    issueMoveCommand(SPEED_FAST, 0, 0);
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
    waitForOK(s);
    boost::asio::write(serial, boost::asio::buffer("M400\n", 5));
    waitForOK("M400");
}

void GrabberController::waitForOK(const string &s) {
    boost::asio::streambuf buf;
    boost::system::error_code ec;
    auto readLen = boost::asio::read_until(serial, buf, '\n', ec);
    if (ec) {
        std::cerr << "\"" << s << "\" -> fails to read back" << std::endl;
    } else {
        std::string info{buffers_begin(buf.data()), buffers_begin(buf.data()) + readLen - 1};
        if (info != "ok") {
            std::cerr << "\"" << s << "\" ->  \"" << info << "\"" << std::endl;
        }
    }
}
