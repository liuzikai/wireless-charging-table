//
// Created by liuzikai on 4/13/21.
//

#include "GrabberController.h"

// TODO: serial device name
GrabberController grabber("/dev/ttyACM1", 115200);

int main() {

    float srcX, srcY, destX, destY;

    while(true) {

        std::cout << "(srcX, srcY, destX, destY), -1 to exit: ";
        std::cin >> srcX;
        if (srcX == -1) break;
        std::cin >> srcY >> destX >> destY;

        grabber.issueGrabberMovement(srcX, srcY, destX, destY);
    }

    return 0;

}