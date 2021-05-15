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

    ::tcflush(serial.lowest_layer().native_handle(), TCIOFLUSH);  // flush input and output
    boost::asio::write(serial, boost::asio::buffer("G28\n", 4));  // home

    // Wait for ok
    while (waitForLine() == "echo:busy: processing") {
        std::cout << "G28 waiting..." << std::endl;
    }
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

    ::tcflush(serial.lowest_layer().native_handle(), TCIFLUSH);  // clear input buffer
    boost::asio::write(serial, boost::asio::buffer(ss.str().c_str(), ss.str().size())); // G0
    boost::asio::write(serial, boost::asio::buffer("M400\n", 5));  // wait
    boost::asio::write(serial, boost::asio::buffer("M114\n", 5));  // echo a line
    waitForLine();  // the line content can be any
}

string GrabberController::waitForLine() {
    boost::asio::streambuf buf;
    boost::system::error_code ec;
    auto readLen = boost::asio::read_until(serial, buf, '\n', ec);
    if (ec) {
        return "";
    } else {
        return {buffers_begin(buf.data()), buffers_begin(buf.data()) + readLen - 1};
    }
}

