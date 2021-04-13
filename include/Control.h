#ifndef CONTROL_H
#define CONTROL_H

// Created by Tingkai Liu 2021-03-29

#define NUM_COILS   3
#define BAUD        19200 // For the serial port /dev/ttyTHS1 (board pin 8&10) or COM (USB)

#include <vector>
#include <map>
#include <string>

// The pin numbers for coil status input in BCM mode (see the datasheet)
static const int coil_pins[NUM_COILS] = {9, 10, 11};

struct Coordinate{
    Coordinate(int x, int y) : x(x), y(y) {}
    
    int x;
    int y;
};

struct Device{
    Coordinate coor;

    // The x, y dimention length of the device
    int x_length;
    int y_length;

    bool chargable;
};

typedef int (*schedule_function)(void);

class Control {

public:
    Control();

    // ~Control();

    enum State{
        WAITING, CALCULATING, MOVING1, MOVING2, ERROR, NUM_STATES
    };

    enum Pin_status{
        IDLE, CHARGING, BLINK, NUM_STATUS  
    };

    /**
     * Launch the all the controls
     * The functions goes into an infinite loop of controlling and never returns
     */
    int launch();


private:
    /********************************* Schduling Controls *****************************/
    schedule_function schedule[NUM_STATES];

    /**
     * The scheduling functions for each states
     * See the document for the work of each state
     */
    int schedule_waiting();
    int schedule_calculating();
    int schedule_moving1();
    int schedule_moving2();
    int schedule_error();
    
    /********************************* GPIO Controls *****************************/
    
    /**
     * Initialize the GPIO pins for collecting wireless charging coil information
     */
    int GPIO_init();

    /**
     * Collect the inputs from all the coils
     * Pin status: 0 means LOW. 1 means HIGH.
     * 1Hz of blink means unchargable device is placed
     * @param   pin_index The index of the pin number (BCM Mode) to check in the pin number array
     * @return  Output as a share variable
     */
    void GPIO_read_inputs(int pin_index);


    /********************************* Mechanical Controls *****************************/
    int move_x();
    int move_y();


    /********************************* Data Fields *****************************/
    State cur_state;
    
    std::string error_message;

    // Device status
    std::map<Coordinate, Device> chargeable;
    std::map<Coordinate, Device> unchargeable;

    Coordinate cur_coil_positions[NUM_COILS];

    // For Wireless Charging Subsystem
    int idle_coil_count;
    Pin_status status[NUM_COILS];

    // The signal that the pin status changed
    // Always set high by pin thread and set low by control (main) thread
    // So no need for lock
    bool pin_needs_handling[NUM_COILS]; 


};


#endif /* #ifndef CONTROL_H */
