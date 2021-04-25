//
// Created by liuzikai on 4/13/21.
//

#include "GrabberController.h"

GrabberController grabber;

int main() {

    float x, y;

    while(true) {

        std::cout << "(X, Y), -1 to reset, -2 to exit: ";
        std::cin >> x;
        if (x == -1) {
            grabber.resetGrabber();
        } else if (x == -2) {
            break;
        } else {
            std::cin >> y;
            grabber.moveGrabber(x, y);
        }
    }
    return 0;

}