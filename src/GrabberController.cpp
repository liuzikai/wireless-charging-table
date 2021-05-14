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
    boost::asio::write(serial, boost::asio::buffer("G28\n", 4));

    // Wait for ok
    string s;
    while ((s = waitForLine()) == "echo:busy: processing") {
        std::cout << "G28 waiting..." << std::endl;
    }
}

void GrabberController::moveGrabber(float x, float y, unsigned speed) {
//    std::cout << "=> " << x << ", " << y << ", " << speed << std::endl;
    issueMoveCommand(speed, x, y, Z_ATTACHED);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
//    std::cout << "=> Done " << x << ", " << y << ", " << speed << std::endl;
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

    // Wait for operation to finish
    if (!waitForReachingTarget(x, y, z)) {
        std::cerr << "M114 failed, fall back to 10s waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

bool GrabberController::waitForReachingTarget(float x, float y, float z) {
    boost::asio::streambuf buf;
    float curX, curY, curZ;
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        clearInputBuffer();
        boost::asio::write(serial, boost::asio::buffer("M400\n", 5));
        boost::asio::write(serial, boost::asio::buffer("M114\n", 5));
        string line = waitForLine();
        return true;
        if (line.empty()) {
            return false;
        } else {
            std::istringstream ss(line);
            char tmp[10];

            ss.read(tmp, 2);
            if (tmp[0] != 'X' || tmp[1] != ':') return false;

            if(!(ss >> curX)) return false;

            ss.read(tmp, 3);
            if (tmp[0] != ' ' || tmp[1] != 'Y' || tmp[2] != ':') return false;

            if(!(ss >> curY)) return false;

            ss.read(tmp, 3);
            if (tmp[0] != ' ' || tmp[1] != 'Z' || tmp[2] != ':') return false;

            if(!(ss >> curZ)) return false;

            /*if ((line = waitForLine()) != "ok") {
                std::cout << "M114 returns " << line << std::endl;
            }*/
        }
    } while (!(std::abs(curX - x) <= 1 && std::abs(curY - y) <= 1 && std::abs(curZ - z) <= 1));

    return true;
}

void GrabberController::serialSendCommand(const string &s) {
    clearInputBuffer();
    boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
    boost::asio::write(serial, boost::asio::buffer("\n", 1));

    string line;
//    if ((line = waitForLine()) != "ok") {
//        std::cout << s << " returns " << line << std::endl;
//    }
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

void GrabberController::clearInputBuffer() {
    ::tcflush(serial.lowest_layer().native_handle(), TCIFLUSH);
}