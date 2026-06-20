// Anteater Electric Racing, 2025

#define BUTTON_DEBOUNCE_MS 500

#include <Arduino.h>
#include <stdint.h>

#include "rtm.h"

// TODO CLEANUP

static bool rtmState = false; // Latching state of RTM based on momentary button
                              // press. True - driving state, false - idle state
static uint32_t lastDebounceTime = 0;

void RTM_ButtonUpdate(bool rtmButton) {

    // static bool lastState;
    // static bool currentState = rtmButton;

    rtmState = rtmButton;

    // if (lastState == false && currentState == true) {
    //     rtmState = true;
    // } else {
    //     rtmState = false;
    // }
    // // reset it at the end
    // lastState = currentState;
}

bool RTM_ButtonState() { return rtmState; }

void RTM_ButtonReset() { rtmState = false; }
