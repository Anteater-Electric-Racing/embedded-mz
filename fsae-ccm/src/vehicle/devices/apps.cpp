// Anteater Electric Racing, 2025

#include "apps.h"
#include "utils/utils.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/faults.h"
#include <arduino_freertos.h>
#include <cmath>

typedef struct {
    float appsReading1_Percentage; // Percentage of pedal travel (0 to 1)
    float appsReading2_Percentage; // Percentage of pedal travel (0 to 1)

    float appsReading1_Voltage; // Voltage reading from the pedal (0 to 3.3V)
    float appsReading2_Voltage; // Voltage reading from the pedal (0 to 5V)

    float apps1RawReading;
    float apps2RawReading;
} APPSData;

static APPSData appsData;
static float appsAlpha;
static TickType_t appsLatestHealthyStateTime =
    0; // Set to 0 when fault not detected

static void checkAndHandleAPPSFault();
static void checkAndHandlePlausibilityFault();

void APPS_Init() {
    appsData.appsReading1_Percentage = 0;
    appsData.appsReading2_Percentage = 0;

    appsData.appsReading1_Voltage = 0;
    appsData.appsReading2_Voltage = 0;

    appsData.apps1RawReading = 0;
    appsData.apps2RawReading = 0;

    appsAlpha = COMPUTE_ALPHA(40.0F);
}

void APPS_UpdateData(uint16_t rawReading1,
                     uint16_t rawReading2) { // changed uint16 from 32

    LOWPASS_FILTER(rawReading1, appsData.apps1RawReading, appsAlpha);
    LOWPASS_FILTER(rawReading2, appsData.apps2RawReading, appsAlpha);
    appsData.appsReading1_Percentage =
        LINEAR_MAP(appsData.apps1RawReading, (float)APPS1_REST_ADC,
                   (float)APPS1_FULL_PCT_ADC, 0.0F, 1.0F);

    appsData.appsReading2_Percentage =
        LINEAR_MAP(appsData.apps2RawReading, (float)APPS2_REST_ADC,
                   (float)APPS2_FULL_PCT_ADC, 0.0F, 1.0F);

    appsData.appsReading1_Voltage =
        ADC_VALUE_TO_VOLTAGE(appsData.apps1RawReading, ADC_VOLTAGE_DIVIDER1);
    appsData.appsReading2_Voltage =
        ADC_VALUE_TO_VOLTAGE(appsData.apps2RawReading, ADC_VOLTAGE_DIVIDER2);
    if (appsData.appsReading1_Voltage < APPS_3V3_MIN) {
        appsData.appsReading1_Voltage = APPS_3V3_MIN;
    } else if (appsData.appsReading1_Voltage > APPS_3V3_MAX) {
        appsData.appsReading1_Voltage = APPS_3V3_MAX;
    }

    if (appsData.appsReading2_Voltage > APPS_3V3_INV_MIN) {
        appsData.appsReading2_Voltage = APPS_3V3_INV_MIN;
    } else if (appsData.appsReading2_Voltage < APPS_3V3_INV_MAX) {
        appsData.appsReading2_Voltage = APPS_3V3_INV_MAX;
    }
    if (appsData.appsReading1_Percentage < 0.0F) {
        appsData.appsReading1_Percentage = 0.0F;
    } else if (appsData.appsReading1_Percentage > 1.0F) {
        appsData.appsReading1_Percentage = 1.0F;
    }

    if (appsData.appsReading2_Percentage < 0.0F) {
        appsData.appsReading2_Percentage = 0.0F;
    } else if (appsData.appsReading2_Percentage > 1.0F) {
        appsData.appsReading2_Percentage = 1.0F;
    }

    checkAndHandleAPPSFault();
    checkAndHandlePlausibilityFault();
}

float APPS_GetAPPSReading() {
    return (appsData.appsReading1_Percentage +
            appsData.appsReading2_Percentage) /
           2.0;
}

float APPS_GetAPPSReading1() { return appsData.appsReading1_Percentage; }

float APPS_GetAPPSReading2() { return appsData.appsReading2_Percentage; }

void APPS_AutoCalibrate() {}

static void checkAndHandleAPPSFault() {
    // Check for open/short circuit
    float difference = abs(appsData.appsReading1_Percentage -
                           appsData.appsReading2_Percentage);

    if (appsData.appsReading1_Voltage < APPS_3V3_FAULT_MIN ||
        appsData.appsReading1_Voltage > APPS_3V3_FAULT_MAX ||
        appsData.appsReading2_Voltage > APPS_3V3_INV_FAULT_MIN ||
        appsData.appsReading2_Voltage < APPS_3V3_INV_FAULT_MAX) {

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsedTicks = now - appsLatestHealthyStateTime;
        TickType_t elapsedMs = elapsedTicks * portTICK_PERIOD_MS;

        if (elapsedMs > APPS_FAULT_TIME_THRESHOLD_MS) {
            Faults_SetFault(FAULT_APPS);
            return;
        }
    } else {
        appsLatestHealthyStateTime = xTaskGetTickCount();
        Faults_ClearFault(FAULT_APPS);
    }

    if (difference > APPS_IMPLAUSABILITY_THRESHOLD) {
        Faults_SetFault(FAULT_APPS);
        return;
    } else {
#if DEBUG_FLAG
        Serial.println("Clearing fault in handle");
#endif
        Faults_ClearFault(FAULT_APPS);
    }
}
/* TODO: Fix the fuckass broken sensor so we can use this*/
static void checkAndHandlePlausibilityFault() {
    // float BSEReading_Front = BSE_GetBSEReading()->bseFront_Reading;
    // float BSEReading_Rear = BSE_GetBSEReading()->bseRear_Reading;

    // float BSEReading = BSEReading_Front;
    // if (BSEReading_Rear > BSEReading_Front) {
    //     BSEReading = BSEReading_Rear;
    // }

#if DEBUG_FLAG
    Serial.print("BSE Reading: ");
    Serial.println(BSEReading);
#endif

    if (APPS_GetAPPSReading() > APPS_BSE_PLAUSABILITY_THROTTLE_THRESHOLD &&
        (BSE_GetBSEAverage() > BRAKE_LIGHT_AVG_THRESHOLD)) {
        Faults_SetFault(FAULT_APPS_BRAKE_PLAUSIBILITY);
    } else {
        if (APPS_GetAPPSReading() < APPS_BSE_PLAUSIBILITY_RESET_THRESHOLD) {
            Faults_ClearFault(FAULT_APPS_BRAKE_PLAUSIBILITY);
        }
    }
}
