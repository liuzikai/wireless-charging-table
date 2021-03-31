
#include "control.h"

// for delay function.
#include <chrono> 
#include <thread>
// for signal handling
#include <signal.h>

#include <JetsonGPIO.h>

using namespace std;

control::control(){
    GPIO_init();
}

int control::GPIO_init(){
    GPIO::setmode(GPIO::BCM); // Set the pin number mode to be BCM

    for (int i = 0; i < NUM_COILS; i++){
        GPIO::setup(coil_pins[i], GPIO::IN);
    }

    return 0;
}

vector<int> control::GPIO_read_inputs(){
    vector<int> ret;

    for (int i = 0; i < NUM_COILS; i++){
        ret.push_back(GPIO::input(coil_pins[i]))
    }

    return ret;
}

