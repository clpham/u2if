#ifndef _INTERFACE_FREQ_METER_H
#define _INTERFACE_FREQ_METER_H

#include "PicoInterfacesBoard.h"
#include "BaseInterface.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/timer.h" // Required for timer_hw->timerawl

#define MAX_FREQ_METERS 4 // Limit concurrent measurements (adjust based on DMA/PWM resources)
#define NUM_EDGE_TIMES 11 // Number of edges to capture for averaging (from picofreq)

// Structure to hold state for one frequency measurement instance
typedef struct {
    bool active;
    uint8_t gpio;
    uint slice_num;
    int dma_chan;
    uint32_t edge_times[NUM_EDGE_TIMES];
} FreqMeterState;

class FreqMeter : public BaseInterface {
public:
    FreqMeter();
    virtual ~FreqMeter();

    CmdStatus process(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus task(uint8_t response[64]); // Likely unused, but required by BaseInterface

private:
    CmdStatus initFreqMeter(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus deinitFreqMeter(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus startFreqMeter(uint8_t const *cmd, uint8_t response[64]);
    CmdStatus getPeriodTicks(uint8_t const *cmd, uint8_t response[64]);

    int findStateByGpio(uint8_t gpio);
    int findFreeState();
    void releaseResources(int state_index);

    FreqMeterState _state[MAX_FREQ_METERS];
    // Keep track of which PWM slices are used by this interface
    uint8_t _used_pwm_slices_mask = 0;
};

#endif // _INTERFACE_FREQ_METER_H
