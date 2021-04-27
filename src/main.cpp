//
// Created by liuzikai on 4/5/21.
//

#include "Vision.h"
#include "Control.h"
#include "ChargerManager.h"
#include "GrabberController.h"

std::unique_ptr<Vision> vision;
std::unique_ptr<Control> control;
std::unique_ptr<ChargerManager> chargerManager;
std::unique_ptr<GrabberController> grabberController;

int main(int argc, char** argv){

    vision = std::make_unique<Vision>();
    chargerManager = std::make_unique<ChargerManager>();
    grabberController = std::make_unique<GrabberController>();
    control = std::make_unique<Control>(vision.get(), chargerManager.get(), grabberController.get());

    control->join();  // this would never return

    return 0;
}
