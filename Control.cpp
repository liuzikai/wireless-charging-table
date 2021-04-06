#include "Control.h"

// for delay function.
#include <chrono> 
#include <thread>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <JetsonGPIO.h>

using namespace std;


static pthread_t pin_thread[NUM_COILS];


// static pthread_mutex_t pin_locks[NUM_COILS]; 


/********************************* Public Functions *****************************/

Control::Control(){
    
    cur_state = WAITING;

    // Fill the schedule functions table
    schedule[0] = &schedule_waiting;
    schedule[1] = &schedule_calculating;
    schedule[2] = &schedule_moving1;
    schedule[3] = &schedule_moving2;
    schedule[4] = &schedule_error;

    // Init the GPIO status
    GPIO_init();

    // Init the coil positions
    cur_coil_positions[0] = Coordinate(0,0);
    cur_coil_positions[1] = Coordinate(0,0);
    cur_coil_positions[2] = Coordinate(0,0);


}

int Control::launch(){
    
    // Launch the thread for pin status checking
    for (int i = 0; i < NUM_COILS; i++){
        if (0 != pthread_create(&pin_thread[i], NULL, GPIO_read_inputs, &i)){
            cur_state = ERROR;
            error_message = "Pin status thread creation failure";
        }
    }

    while(1){
        schedule_function[cur_state]();
        sleep(1); // Sleep for 1s
    }

    return 0; // Should never reach here
}


/********************************* Schduling Controls *****************************/

int schedule_waiting(){
    
    bool new_wireless_status = false;
    bool new_vision_status = false;

    // Pull for the wireless charging status
    for (int i = 0; i < NUM_COILS; i++){
        new_wireless_status |= pin_needs_handling[i];
    }

    // Pull for vision status

    // Handle the changes
    // Wireless charging has priority since a available number of device may change
    if (new_wireless_status){
        
        
        cur_state = CALCULATING;
    }

    if (new_vision_status){

        // Ignore the new device if no idel coils left
    }

    return 0;
}

int schedule_calculating(){
    return 0;
}

int schedule_moving1(){
    return 0;
}

int schedule_moving2(){
    return 0;
}

int schedule_error(){
    printf("Error: %s\n", error_message.c_str());
    return 0;
}

/********************************* GPIO Controls *****************************/

int Control::GPIO_init(){
    GPIO::setmode(GPIO::BCM); // Set the pin number mode to be BCM

    for (int i = 0; i < NUM_COILS; i++){
        GPIO::setup(coil_pins[i], GPIO::IN);
        old_status[i] = IDLE;
        pin_needs_handling = false;
    }

    return 0;
}

void Control::GPIO_read_inputs(int pin_index){
    
    while(1){
        usleep(10000); // sleep for 10ms TODO: longer?

        if (pin_needs_handling[pin_index]) continue;

        int cur_input = GPIO::input(coil_pins[pin_index]);
        Pin_status cur_status;

        // Check for whether it is blinking TODO:


        if (cur_status == status[pin_index]) continue;

        // New status occured! Notify the control thread
        pin_needs_handling[pin_index] = true;
        status[pin_index] = cur_status;
    }

    return;
}


/********************************* Mechanical Controls *****************************/

