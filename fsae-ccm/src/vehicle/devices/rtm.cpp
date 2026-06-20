// Anteater Electric Racing, 2025

#define BUTTON_DEBOUNCE_MS 500

#include <Arduino.h>
#include <stdint.h>

#include "rtm.h"

// TODO CLEANUP

static bool rtmState = false; // Latching state of RTM based on momentary button

void RTM_ButtonUpdate(bool rtmButton) {

    static bool lastState;

    // rtmState = rtmButton;

    if (lastState == false && rtmButton == true) {
        rtmState = true;
    }
    // reset it at the end
    lastState = rtmButton;
}

bool RTM_ButtonState() { return rtmState; }

void RTM_ButtonReset() { rtmState = false; }
