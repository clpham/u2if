// File: firmware/source/interfaces/FreqCounter.h
#ifndef _INTERFACE_FREQCOUNTER_H
#define _INTERFACE_FREQCOUNTER_H

#include "PicoInterfacesBoard.h"
#include "BaseInterface.h"
#include "hardware/pio.h"
#include <map> // Or use a fixed array if preferred

//// Forward declaration for the PIO program
////extern const struct pio_program freq_counter_program;

#define MAX_FREQ_COUNTERS 8 // Max PIO state machines

struct FreqCounterInfo {
    uint pin;
    PIO pio;
    uint sm;
    uint pio_offset;
    bool active;
};

class FreqCounter : public BaseInterface {
public:
    FreqCounter();
    virtual ~FreqCounter();

    CmdStatus process(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus task(uint8_t response[64]) { (void)response; return CmdStatus::NOT_CONCERNED; } // No background task needed

protected:
    CmdStatus initCounter(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus deinitCounter(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus getMeasurement(uint8_t const *cmd, uint8_t response[64]);

private:
    bool findFreeSM(PIO *pio, uint *sm);
    void releaseSM(PIO pio, uint sm);
    int findCounterIndex(uint pin);

    FreqCounterInfo counters[MAX_FREQ_COUNTERS];
    bool pio_sm_claimed[NUM_PIOS][NUM_PIO_STATE_MACHINES]; // Track claimed SMs
    uint pio_program_offset[NUM_PIOS]; // Store offset if program loaded
    bool pio_program_loaded[NUM_PIOS];
};

#endif // _INTERFACE_FREQCOUNTER_H
