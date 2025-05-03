// File: firmware/source/interfaces/FreqCounter.cpp
#include "FreqCounter.h"
#include "freq_counter.pio.h" // Generated header
#include "hardware/clocks.h"
#include <stdio.h> // For debugging printf if needed

FreqCounter::FreqCounter() {
    setInterfaceState(InterfaceState::INTIALIZED);
    for (int i = 0; i < MAX_FREQ_COUNTERS; ++i) {
        counters[i].active = false;
        counters[i].pin = -1;
    }
    for (uint pio_idx = 0; pio_idx < NUM_PIOS; ++pio_idx) {
        pio_program_loaded[pio_idx] = false;
        pio_program_offset[pio_idx] = 0; // Or some invalid value
        for (uint sm_idx = 0; sm_idx < NUM_PIO_STATE_MACHINES; ++sm_idx) {
            pio_sm_claimed[pio_idx][sm_idx] = false;
        }
    }
}

FreqCounter::~FreqCounter() {
    // Deinitialize all active counters on destruction
    for (int i = 0; i < MAX_FREQ_COUNTERS; ++i) {
        if (counters[i].active) {
            pio_sm_set_enabled(counters[i].pio, counters[i].sm, false);
            releaseSM(counters[i].pio, counters[i].sm);
            // Maybe remove program if last user? Handled in releaseSM
        }
    }
}

CmdStatus FreqCounter::process(uint8_t const *cmd, uint8_t response[64]) {
    CmdStatus status = CmdStatus::NOT_CONCERNED;

    switch (cmd[0]) {
        case Report::ID::FREQ_COUNTER_INIT:
            status = initCounter(cmd, response);
            break;
        case Report::ID::FREQ_COUNTER_DEINIT:
            status = deinitCounter(cmd, response);
            break;
        case Report::ID::FREQ_COUNTER_GET_MEASUREMENT:
            status = getMeasurement(cmd, response);
            break;
        default:
            status = CmdStatus::NOT_CONCERNED;
            break;
    }
    return status;
}

bool FreqCounter::findFreeSM(PIO *pio_out, uint *sm_out) {
    for (uint pio_idx = 0; pio_idx < NUM_PIOS; ++pio_idx) {
        PIO pio = pio_idx == 0 ? pio0 : pio1;
        for (uint sm_idx = 0; sm_idx < NUM_PIO_STATE_MACHINES; ++sm_idx) {
            if (!pio_sm_claimed[pio_idx][sm_idx]) {
                // Claim it
                pio_sm_claimed[pio_idx][sm_idx] = true;
                *pio_out = pio;
                *sm_out = sm_idx;

                // Load program if not already loaded on this PIO instance
                if (!pio_program_loaded[pio_idx]) {
                    if (pio_can_add_program(pio, &freq_counter_program)) {
                        pio_program_offset[pio_idx] = pio_add_program(pio, &freq_counter_program);
                        pio_program_loaded[pio_idx] = true;
                    } else {
                        // Failed to load program, release SM and return error
                        pio_sm_claimed[pio_idx][sm_idx] = false;
                        return false;
                    }
                }
                return true; // Found and claimed
            }
        }
    }
    return false; // No free state machine found
}

void FreqCounter::releaseSM(PIO pio, uint sm) {
    uint pio_idx = pio_get_index(pio);
    if (sm < NUM_PIO_STATE_MACHINES && pio_sm_claimed[pio_idx][sm]) {
        pio_sm_claimed[pio_idx][sm] = false;

        // Check if this was the last user of the program on this PIO
        bool program_in_use = false;
        for (uint sm_idx = 0; sm_idx < NUM_PIO_STATE_MACHINES; ++sm_idx) {
            if (pio_sm_claimed[pio_idx][sm_idx]) {
                program_in_use = true;
                break;
            }
        }

        if (!program_in_use && pio_program_loaded[pio_idx]) {
            pio_remove_program(pio, &freq_counter_program, pio_program_offset[pio_idx]);
            pio_program_loaded[pio_idx] = false;
        }
    }
}

 int FreqCounter::findCounterIndex(uint pin) {
     for (int i = 0; i < MAX_FREQ_COUNTERS; ++i) {
         if (counters[i].active && counters[i].pin == pin) {
             return i;
         }
     }
     return -1; // Not found
 }


CmdStatus FreqCounter::initCounter(uint8_t const *cmd, uint8_t response[64]) {
    uint8_t pin = cmd[1];
    response[2] = pin; // Echo pin number

    // Check if pin is already used
    if (findCounterIndex(pin) != -1) {
         response[3] = 0x02; // Error: Pin already used
         return CmdStatus::NOK;
    }

    // Find a free slot in our counter array
    int free_idx = -1;
    for (int i = 0; i < MAX_FREQ_COUNTERS; ++i) {
        if (!counters[i].active) {
            free_idx = i;
            break;
        }
    }
    if (free_idx == -1) {
         response[3] = 0x01; // Error: No counter slots available (should match MAX_FREQ_COUNTERS)
         return CmdStatus::NOK;
    }


    PIO pio;
    uint sm;
    if (!findFreeSM(&pio, &sm)) {
        response[3] = 0x01; // Error: No PIO SM available
        return CmdStatus::NOK;
    }

    // Store info
    counters[free_idx].pin = pin;
    counters[free_idx].pio = pio;
    counters[free_idx].sm = sm;
    counters[free_idx].pio_offset = pio_program_offset[pio_get_index(pio)];
    counters[free_idx].active = true;

    // Initialize the state machine
    freq_counter_program_init(pio, sm, counters[free_idx].pio_offset, pin);

    //clpham:
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    convertUInt32ToBytes(sys_clk_hz, &response[3]); // Bytes 3, 4, 5, 6
    return CmdStatus::OK;
}

CmdStatus FreqCounter::deinitCounter(uint8_t const *cmd, uint8_t response[64]) {
    uint8_t pin = cmd[1];
    response[2] = pin;

    int idx = findCounterIndex(pin);
    if (idx == -1) {
        // Pin not found or not initialized for frequency counting
        return CmdStatus::NOK; // Or OK if deiniting non-existent is fine? Let's say NOK.
    }

    // Disable and release resources
    pio_sm_set_enabled(counters[idx].pio, counters[idx].sm, false);
    releaseSM(counters[idx].pio, counters[idx].sm);
    counters[idx].active = false;
    counters[idx].pin = -1; // Mark as inactive

    return CmdStatus::OK;
}

CmdStatus FreqCounter::getMeasurement(uint8_t const *cmd, uint8_t response[64]) {
    uint8_t pin = cmd[1];
    response[2] = pin;

    int idx = findCounterIndex(pin);
    if (idx == -1 || !counters[idx].active) {
        // Pin not initialized or inactive
        return CmdStatus::NOK;
    }

    uint32_t high_rem, low_rem;
    // TODO: Add timeout mechanism here? For now, it blocks.
    freq_counter_get_measurement_blocking(counters[idx].pio, counters[idx].sm, &high_rem, &low_rem);

    // Calculate actual cycles (firmware side)
    uint32_t high_cycles = high_rem;
    uint32_t low_cycles = low_rem;

    // Pack results into response buffer (Little Endian)
    convertUInt32ToBytes(high_cycles, &response[3]); // Bytes 3, 4, 5, 6
    convertUInt32ToBytes(low_cycles, &response[7]);  // Bytes 7, 8, 9, 10

    return CmdStatus::OK;
}
