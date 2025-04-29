#include "FreqMeter.h"
#include "pico/stdlib.h" // For memset
#include <stdio.h> // For debugging if needed (remove later)
#include <cstring>

// External functions defined in main.cpp for PWM slice management
extern bool is_pwm_slice_busy(uint slice_num);
extern bool allocate_pwm_slice(uint slice_num);
extern void release_pwm_slice(uint slice_num);


FreqMeter::FreqMeter() {
    for (int i = 0; i < MAX_FREQ_METERS; ++i) {
        _state[i].active = false;
        _state[i].dma_chan = -1; // Indicate DMA not claimed
    }
    setInterfaceState(InterfaceState::INTIALIZED);
}

FreqMeter::~FreqMeter() {
    // Release all resources on destruction
    for (int i = 0; i < MAX_FREQ_METERS; ++i) {
        if (_state[i].active) {
            releaseResources(i);
        }
    }
}

CmdStatus FreqMeter::process(uint8_t const *cmd, uint8_t response[64]) {
    CmdStatus status = CmdStatus::NOT_CONCERNED;

    switch (cmd[0]) {
        case Report::ID::FREQ_METER_INIT:
            status = initFreqMeter(cmd, response);
            break;
        case Report::ID::FREQ_METER_DEINIT:
            status = deinitFreqMeter(cmd, response);
            break;
        case Report::ID::FREQ_METER_START:
            status = startFreqMeter(cmd, response);
            break;
        case Report::ID::FREQ_METER_GET_PERIOD_TICKS:
            status = getPeriodTicks(cmd, response);
            break;
        default:
            status = CmdStatus::NOT_CONCERNED;
            break;
    }
    return status;
}

// Task function - not actively used for frequency meter unless background processing needed
CmdStatus FreqMeter::task(uint8_t response[64]) {
    (void)response; // Unused parameter
    return CmdStatus::NOT_CONCERNED;
}

// Find an existing state entry by GPIO pin
int FreqMeter::findStateByGpio(uint8_t gpio) {
    for (int i = 0; i < MAX_FREQ_METERS; ++i) {
        if (_state[i].active && _state[i].gpio == gpio) {
            return i;
        }
    }
    return -1; // Not found
}

// Find the first inactive state entry
int FreqMeter::findFreeState() {
    for (int i = 0; i < MAX_FREQ_METERS; ++i) {
        if (!_state[i].active) {
            return i;
        }
    }
    return -1; // No free slots
}

// Release DMA and PWM resources for a given state index
void FreqMeter::releaseResources(int state_index) {
    if (state_index < 0 || state_index >= MAX_FREQ_METERS || !_state[state_index].active) {
        return;
    }

    FreqMeterState &st = _state[state_index];

    // Disable PWM
    pwm_set_enabled(st.slice_num, false);

    // Abort and release DMA
    if (st.dma_chan >= 0) {
        if (dma_channel_is_claimed(st.dma_chan)) {
             dma_channel_abort(st.dma_chan);
             dma_channel_unclaim(st.dma_chan);
        }
        st.dma_chan = -1;
    }

    // Mark PWM slice as free for this interface
    _used_pwm_slices_mask &= ~(1u << st.slice_num);

    // --- Resource Release ---
    // Release the PWM slice in the global manager
    release_pwm_slice(st.slice_num);
    // --- End Resource Release ---


    // Mark state as inactive
    st.active = false;
    // Optionally reset GPIO function if needed, but deinit might handle this
    // gpio_set_function(st.gpio, GPIO_FUNC_SIO);
}


CmdStatus FreqMeter::initFreqMeter(uint8_t const *cmd, uint8_t response[64]) {
    const uint8_t gpio = cmd[1];
    response[2] = gpio; // Echo GPIO back in response
    response[3] = 0x00; // Default no error

    // Check if GPIO already initialized for frequency measurement
    if (findStateByGpio(gpio) != -1) {
        response[3] = 0x01; // Error: Pin already initialized by this interface
        return CmdStatus::NOK;
    }

    // Find a free state slot
    int state_index = findFreeState();
    if (state_index == -1) {
        response[3] = 0x03; // Error: Max concurrent measurements reached
        return CmdStatus::NOK;
    }

    // Check PWM slice availability
    // Note: Assumes PWM channel B is used for input counting, like picofreq
    if (pwm_gpio_to_channel(gpio) != PWM_CHAN_B) {
         // Or handle channel A if necessary, but stick to B for consistency with picofreq
         response[3] = 0x04; // Error: Pin must map to PWM Channel B
         return CmdStatus::NOK;
    }
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Check if this slice is already used by *this* interface
    if ((_used_pwm_slices_mask >> slice_num) & 1u) {
         response[3] = 0x01; // Error: Slice already used by freq meter
         return CmdStatus::NOK;
    }

    // TODO: Need a way to check if the slice is used by the PWM *output* interface
    // This requires coordination between Pwm.cpp and FreqMeter.cpp, possibly via a shared resource manager.
    // For now, we assume it's free if not used by FreqMeter.
    // --- Resource Allocation ---
    if (!allocate_pwm_slice(slice_num)) {
        response[3] = 0x01; // Error: Slice is busy (used by PWM output or another FreqMeter)
        return CmdStatus::NOK;
    }
    // --- End Resource Allocation ---

    // Claim DMA channel
    int dma_chan = dma_claim_unused_channel(true);
    if (dma_chan < 0) {
        response[3] = 0x02; // Error: No free DMA channel
        return CmdStatus::NOK;
    }

    // --- Configure PWM for edge counting (adapted from picofreq_init) ---
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    // Count rising edges on channel B
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1); // Count every edge
    pwm_init(slice_num, &cfg, false); // Start disabled
    //pwm_set_wrap(slice_num, 65535); // Max wrap value
    pwm_set_wrap(slice_num, 0); // clpham: from Bus Pirate 5 picofreq.c, edge_timer_init()

    // --- Configure DMA for edge timing (adapted from edge_timer_init) ---
    FreqMeterState &st = _state[state_index];
    dma_channel_config dma_cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32); // Transfer 32-bit timer values
    channel_config_set_read_increment(&dma_cfg, false);          // Read from fixed timer address
    channel_config_set_write_increment(&dma_cfg, true);          // Write sequentially to buffer
    channel_config_set_dreq(&dma_cfg, pwm_get_dreq(slice_num));   // Trigger DMA on PWM event (wrap/compare) - should trigger on edge count? Check RP2040 datasheet. DREQ triggers when counter matches CC register. Let's set CC=0 to trigger on first edge? Or maybe DREQ triggers anyway on input mode? Let's assume DREQ works on input edge.

    dma_channel_configure(
        dma_chan,
        &dma_cfg,
        st.edge_times,          // Write address (start of buffer)
        &timer_hw->timerawl,    // Read address (32-bit timer low word - 1us resolution)
        NUM_EDGE_TIMES,         // Number of transfers
        false                   // Don't start yet
    );

    // Store state
    st.active = true;
    st.gpio = gpio;
    st.slice_num = slice_num;
    st.dma_chan = dma_chan;
    _used_pwm_slices_mask |= (1u << slice_num); // Mark slice as used by this interface

    return CmdStatus::OK;
}

CmdStatus FreqMeter::deinitFreqMeter(uint8_t const *cmd, uint8_t response[64]) {
    const uint8_t gpio = cmd[1];
    response[2] = gpio;

    int state_index = findStateByGpio(gpio);
    if (state_index == -1) {
        // Not initialized or already deinitialized, consider it OK
        return CmdStatus::OK;
    }

    releaseResources(state_index); // This now handles releasing the slice

    // Optional: Reset GPIO function if desired
    gpio_set_function(gpio, GPIO_FUNC_SIO);
    gpio_disable_pulls(gpio); // Remove any pulls if set elsewhere

    return CmdStatus::OK;
}

CmdStatus FreqMeter::startFreqMeter(uint8_t const *cmd, uint8_t response[64]) {
    const uint8_t gpio = cmd[1];
    response[2] = gpio;

    int state_index = findStateByGpio(gpio);
    if (state_index == -1) {
        response[3] = 0x01; // Error: Not initialized
        return CmdStatus::NOK;
    }

    FreqMeterState &st = _state[state_index];

    // Ensure previous DMA is stopped if any
    if (dma_channel_is_busy(st.dma_chan)) {
         dma_channel_abort(st.dma_chan);
    }

    // Clear the buffer
    memset(st.edge_times, 0, sizeof(st.edge_times));

    // --- Start DMA (adapted from edge_timer_start) ---
    // Re-configure destination address and transfer count for safety
    dma_channel_set_write_addr(st.dma_chan, st.edge_times, false);
    dma_channel_set_trans_count(st.dma_chan, NUM_EDGE_TIMES, false);
    // Trigger the DMA transfer (starts waiting for DREQ)
    dma_channel_start(st.dma_chan);


    // Reset and enable PWM counter
    pwm_set_counter(st.slice_num, 0); // Reset edge count
    pwm_set_enabled(st.slice_num, true);

    return CmdStatus::OK;
}

CmdStatus FreqMeter::getPeriodTicks(uint8_t const *cmd, uint8_t response[64]) {
    const uint8_t gpio = cmd[1];
    response[2] = gpio;

    int state_index = findStateByGpio(gpio);
    if (state_index == -1) {
        response[3] = 0x01; // Error: Not initialized
        return CmdStatus::NOK;
    }

    FreqMeterState &st = _state[state_index];

    // --- Get average period (adapted from edge_timer_value) ---
    // Stop the DMA transfer to read the current state
    // Note: This might cut off a measurement mid-way if called too soon.
    // Consider adding a check or timeout? For now, just abort and read.
     if (dma_channel_is_busy(st.dma_chan)) {
        dma_channel_abort(st.dma_chan);
     }

    // Disable PWM counter while processing
    pwm_set_enabled(st.slice_num, false);

    uint i = 1;
    uint32_t n = 0;
    uint64_t total_ticks = 0; // Use 64-bit for sum to avoid overflow
    uint valid_periods = 0;

    // Calculate differences between consecutive edge times
    while (i < NUM_EDGE_TIMES && st.edge_times[i] != 0) {
        // Check for timer wrap-around (unlikely for short periods, but possible)
        if (st.edge_times[i] > st.edge_times[i-1]) {
            n = st.edge_times[i] - st.edge_times[i-1];
            total_ticks += n;
            valid_periods++;
        } else {
             // Timer wrapped or bad data, skip this interval
        }
        i++;
    }

    uint32_t avg_period_ticks = 0;
    if (valid_periods > 0) {
        avg_period_ticks = static_cast<uint32_t>(total_ticks / valid_periods);
    } else {
        // No valid periods measured (e.g., no edges detected, or only one edge)
        response[3] = 0x02; // Measurement error/timeout
        // Optionally clear edge_times buffer here?
        // memset(st.edge_times, 0, sizeof(st.edge_times));
        return CmdStatus::NOK;
    }

    // Write average period ticks to response buffer (Little Endian)
    convertUInt32ToBytes(avg_period_ticks, &response[3]);

    // Optionally clear edge_times buffer after reading?
    // memset(st.edge_times, 0, sizeof(st.edge_times));

    return CmdStatus::OK;
}
