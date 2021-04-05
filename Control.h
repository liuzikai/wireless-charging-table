#ifndef CONTROL_H
#define CONTROL_H

#define NUM_COILS   3

#include <vector>

// The pin numbers for coil status input in BCM mode (see the datasheet)
static const int coil_pins[NUM_COILS] = {9, 10, 11};


class Control {

public:
    Control();

    // ~Control();

    /**
     * Initialize the GPIO pins for collecting wireless charging coil information
     */
    int GPIO_init();

    /**
     * Collect the inputs from all the coils
     * Pin status: 0 means LOW. 1 means HIGH.
     * @return  The vector holding the status for all the coil pins, in the same order as
     *          pin assignment array above.
     */
    std::vector<int> GPIO_read_inputs();


};


#endif /* #ifndef CONTROL_H */
