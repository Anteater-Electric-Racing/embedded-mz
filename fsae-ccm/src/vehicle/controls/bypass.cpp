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
 * Logic (signal polarity):
 * - Feedback LOW  => NO FAULT
 * - Feedback HIGH => FAULT (needs bypass)
 *
 * Behavior:
 * - On startup: if no fault (LOW) -> ensure bypass OFF and exit startup.
 *   If fault (HIGH) during startup and previous latched state was healthy,
 *   allow a startup bypass; otherwise keep bypass OFF.
 * - Normal: enable bypass when feedback==HIGH; when feedback==LOW debounce
 *   and then disable bypass.
 */

void Bypass_UpdateState() { EEPROM.update(fault_address, feedbackStatus); }

void Bypass_TSSI() {
    feedbackStatus = digitalRead(TSSI_FEEDBACK_PIN);

    Serial.print("in startup? : ");
    Serial.print(startup ? "TRUE" : "FALSE");
    Serial.print(" | feedbackStatus = ");
    Serial.print(feedbackStatus == LOW ? "NO FAULT" : "FAULT");
    Serial.print(" | ");
    Serial.print("latched fault from prev ON = ");
    Serial.print(is_fault == LOW ? "NO FAULT ---- BYPASSING"
                                : "FAULT ---- DON'T BYPASS");
    Serial.print("\r");

    // I get feedback as FAULT always on startup
    //issue is that no fault feedback happens until I click the latchboard

    if (startup) {
        if (feedbackStatus == LOW) {
            // No fault: ensure bypass OFF and exit startup
            digitalWrite(TSSI_BYPASS_PIN, LOW);
            startup = false;
        } else {
            // Fault during startup
            // If we were healthy before (is_fault == LOW) allow a startup bypass,
            // otherwise keep bypass OFF.
            if (is_fault == LOW) {
                digitalWrite(TSSI_BYPASS_PIN, HIGH); // bypass this startup fault
            } else {
                digitalWrite(TSSI_BYPASS_PIN, LOW); // don't bypass
            }
        }
    } else {
        // Normal operation
        if (feedbackStatus == HIGH) {
            // Fault detected -> enable bypass
            digitalWrite(TSSI_BYPASS_PIN, HIGH);
        } else {
            // No fault - debounce before turning OFF bypass
            vTaskDelay(pdMS_TO_TICKS(100));
            // re-read the feedback after debounce
            feedbackStatus = digitalRead(TSSI_FEEDBACK_PIN);
            if (feedbackStatus == LOW) {
                digitalWrite(TSSI_BYPASS_PIN, LOW); // turn OFF bypass
            }
        }
    }

    Bypass_UpdateState();
}

// TO FIX
void Bypass_TSSI_Full() {
    bool fb = digitalRead(TSSI_FEEDBACK_PIN);
    bool imdStatus = (IMD_GetInfo()->status == 0x200) ? LOW : HIGH;

    if (fb == HIGH && imdStatus == LOW) {
        // Fault present and IMD clear -> enable bypass
        digitalWrite(TSSI_BYPASS_PIN, HIGH);
    } else if (fb == LOW) {
        // No fault: debounce then disable bypass
        vTaskDelay(pdMS_TO_TICKS(100));
        fb = digitalRead(TSSI_FEEDBACK_PIN);
        if (fb == LOW) {
            digitalWrite(TSSI_BYPASS_PIN, LOW);
        }
    }
}
