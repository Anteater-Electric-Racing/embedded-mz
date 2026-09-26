/*
TSSI Bypass Control
Description:

*/

#include <Arduino.h>
#include <arduino_freertos.h>
using namespace arduino;

#include "bypass.h"
#include "vehicle/comms/bus.h"

static bool startup;         // starts high
static bool is_fault = HIGH; // assume good
static bool feedbackStatus;

void Bypass_Init() {
    pinMode(TSSI_BYPASS_PIN, OUTPUT);
    pinMode(TSSI_FEEDBACK_PIN, INPUT);

    // Default state: Bypass OFF

    // digitalWrite(TSSI_BYPASS_PIN, LOW);
    startup = true;
    is_fault = EEPROM.read(fault_address);
}

/**
 * Logic:
 * Feedback LOW (faults need to be bypassed) & IMD Status Low ->
 * Bypass ON (LOW) Feedback HIGH (faults are clear) -> Bypass OFF (HIGH) after
 * delay
 */

void Bypass_UpdateState() { EEPROM.update(fault_address, feedbackStatus); }

void Bypass_TSSI() {
    feedbackStatus = digitalRead(TSSI_FEEDBACK_PIN);
    bool imdFaulted = (IMD_GetInfo()->status == 0x200) ? LOW : HIGH;
    bool actualFault = imdFaulted; // || bmsFaulted;
    if (startup) {
        if (feedbackStatus == HIGH) {
            digitalWrite(TSSI_BYPASS_PIN, HIGH);
            startup = false;
        } else if (feedbackStatus == LOW) {
            // Fault during startup
            if (is_fault == LOW) {
                // Serial.println("trigger2");
                // Was in fault state before - don't bypass
                digitalWrite(TSSI_BYPASS_PIN, HIGH);
            } else {
                // Serial.println("trigger3");
                // Was healthy before - bypass this startup fault
                digitalWrite(TSSI_BYPASS_PIN, LOW);
            }
        }
    } else {
        // Normal operation
        if (feedbackStatus == LOW) {
            Serial.println("trigger4");
            // Fault detected
            digitalWrite(TSSI_BYPASS_PIN, HIGH);
        } else {
            // No fault - delay and debounce before turning off bypass
            vTaskDelay(pdMS_TO_TICKS(100));
            if (feedbackStatus == HIGH) {
                // Serial.println("trigger5");
                digitalWrite(TSSI_BYPASS_PIN, HIGH);
            }
        }
    }

    Bypass_UpdateState();
}

// TO FIX same logic as eariler
/**
 * need a bool called actualFault, if its any error except in startup mode
 * (for IMD, BMS), actual fault should be true Question: what faults should
 * be detected as a part of this for BMS? Any fault at ALL?
 *
 */
void Bypass_TSSI_Full() {
    bool feedbackStatus = digitalRead(TSSI_FEEDBACK_PIN);
    bool imdStatus = (IMD_GetInfo()->status == 0x200) ? LOW : HIGH;

    if (feedbackStatus == LOW && imdStatus == LOW) {
        // Feedback is LOW, turn bypass ON immediately
        // Serial.println("trigger6");
        digitalWrite(TSSI_BYPASS_PIN, HIGH);
    } else {
        // Feedback is HIGH, wait a bit then turn OFF
        vTaskDelay(pdMS_TO_TICKS(100));

        // Double check if feedback is still HIGH before switching off
        if (feedbackStatus == HIGH) {
            // Serial.println("trigger7");
            digitalWrite(TSSI_BYPASS_PIN, LOW);
        }
    }
}
