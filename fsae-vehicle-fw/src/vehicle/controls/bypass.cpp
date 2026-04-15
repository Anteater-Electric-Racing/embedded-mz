/*
TSSI Bypass Control
Description:

*/

#include <Arduino.h>
#include <arduino_freertos.h>
using namespace arduino;

#include "bypass.h"
#include "vehicle/comms/bus.h"

void Bypass_Init() {
    pinMode(TSSI_BYPASS_PIN, OUTPUT);
    pinMode(TSSI_FEEDBACK_PIN, INPUT);

    // Default state: Bypass OFF
    digitalWrite(TSSI_BYPASS_PIN, LOW);
}

/**
 * Logic:
 * Feedback LOW  & IMD Status Low -> Bypass ON (High)
 * Feedback HIGH -> Bypass OFF (Low) after delay
 *
 */
void Bypass_TSSI_Control() {
    bool feedbackStatus = digitalRead(TSSI_FEEDBACK_PIN);
    bool imdStatus = (IMD_GetInfo()->status == 0x200) ? LOW : HIGH;

    if (feedbackStatus == LOW && imdStatus == LOW) {
        // Feedback is LOW, turn bypass ON immediately
        digitalWrite(TSSI_BYPASS_PIN, HIGH);
    } else {
        // Feedback is HIGH, wait a bit then turn OFF
        vTaskDelay(pdMS_TO_TICKS(100));

        // Double check if feedback is still HIGH before switching off
        if (feedbackStatus == HIGH) {
            digitalWrite(TSSI_BYPASS_PIN, LOW);
        }
    }
}
