// Anteater Electric Racing, 2025

#include "apps.h"
#include "utils/utils.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/faults.h"
#include <arduino_freertos.h>
#include <cmath>
#include <EEPROM.h>

typedef struct {
    float appsReading1_Percentage; // Percentage of pedal travel (0 to 1)
    float appsReading2_Percentage; // Percentage of pedal travel (0 to 1)

    float appsReading1_Voltage; // Voltage reading from the pedal (0 to 3.3V)
    float appsReading2_Voltage; // Voltage reading from the pedal (0 to 5V)

    float apps1RawReading;
    float apps2RawReading;
} APPSData;

static float restingAPPS1_ADC;
static float restingAPPS2_ADC;
static float fullAPPS1_ADC;
static float fullAPPS2_ADC;
static uint8_t apps1FullWritten;
static uint8_t apps2FullWritten;
static uint8_t apps1FullWrittenExpected = 67;
static uint8_t apps2FullWrittenExpected = 42;

static APPSData appsData;
static float appsAlpha;
static TickType_t appsLatestHealthyStateTimeAPPSRange =
    0; // Set to 0 when fault not detected

static TickType_t appsLatestHealthyStateTimeAPPSDifference =
    0; // Set to 0 when fault not detected


static void checkAndHandleAPPSFault();
static void checkAndHandlePlausibilityFault();


void APPS_Calibrate_Rest() {

    // collect 50 samples of apps adc
    uint8_t i = 0;
    uint32_t totalADC1 = 0;
    uint32_t totalADC2 = 0;
    uint8_t cycles = 50;
    uint16_t rawReading1;
    uint16_t rawReading2;
    uint16_t processedReading1 = ADC_GetAPPS1Value();
    uint16_t processedReading2 = ADC_GetAPPS2Value();
    for(; i < cycles; ++i){
        rawReading1 = ADC_GetAPPS1Value();
        rawReading2 = ADC_GetAPPS2Value();
        LOWPASS_FILTER(rawReading1, processedReading1, appsAlpha);
        LOWPASS_FILTER(rawReading2, processedReading2, appsAlpha);
        totalADC1 += processedReading1;
        totalADC2 += processedReading2;
    }

    // get average
    uint16_t averageADC1 = (totalADC1 / cycles);
    uint16_t averageADC2 = (totalADC2 / cycles);

    // compare average to expected value and update if within reasonable range
    if (abs(averageADC1 - APPS1_REST_ADC) < APPS_ADC_DIFF_BUFF){
        restingAPPS1_ADC = averageADC1;
    } else {
        restingAPPS1_ADC = APPS1_REST_ADC;
    }

    if (abs(averageADC2 - APPS2_REST_ADC) < APPS_ADC_DIFF_BUFF){
        // EEPROM.update(apps2Rest_address, averageADC2);
        restingAPPS2_ADC = averageADC2;
    } else {
        restingAPPS2_ADC = APPS2_REST_ADC;
    }

#if APPS_CALIBRATION_DEBUG
    Serial.print("Resting APPS1 ADC: ");
    Serial.println(restingAPPS1_ADC);
    Serial.print("APPS1 Average Resting ADC : ");
    Serial.println(averageADC1);
    Serial.print("APPS1 Resting Comparison : ");
    Serial.println((averageADC1 == APPS1_REST_ADC));

    Serial.print("Resting APPS2 ADC: ");
    Serial.println(restingAPPS2_ADC);
    Serial.print("APPS2 Average Resting ADC : ");
    Serial.println(averageADC2);
    Serial.print("APPS2 Resting Comparison : ");
    Serial.println((averageADC2 == APPS2_REST_ADC));
# endif

    EEPROM.get(apps1FullWritten_address, apps1FullWritten);
    EEPROM.get(apps2FullWritten_address, apps2FullWritten);

    if (apps1FullWritten != apps1FullWrittenExpected){
        EEPROM.put(apps1Full_address, APPS1_FULL_PCT_ADC);
        EEPROM.put(apps1FullWritten_address, apps1FullWrittenExpected);
    }

    if (apps2FullWritten != apps2FullWrittenExpected){
        EEPROM.put(apps2Full_address, APPS2_FULL_PCT_ADC);
        EEPROM.put(apps2FullWritten_address, apps2FullWrittenExpected);
    }

    EEPROM.get(apps1Full_address, fullAPPS1_ADC);
    EEPROM.get(apps2Full_address, fullAPPS2_ADC);

#if APPS_CALIBRATION_DEBUG
    Serial.print("Full APPS1 ADC: ");
    Serial.println(fullAPPS1_ADC);
    Serial.print("APPS1 Original Full ADC : ");
    Serial.println(APPS1_FULL_PCT_ADC);
    Serial.print("APPS1 Comparison : ");
    Serial.println((averageADC1 == APPS1_FULL_PCT_ADC));

    Serial.print("Resting APPS2 ADC: ");
    Serial.println(fullAPPS2_ADC);
    Serial.print("APPS2 Original ADC : ");
    Serial.println(APPS2_FULL_PCT_ADC);
    Serial.print("APPS2 Comparison : ");
    Serial.println((fullAPPS2_ADC == APPS2_FULL_PCT_ADC));

# endif

}

void APPS_Calibrate_Full(){
    // if rtm button on
    // read 50 samples & write to eeprom

    // if never written to eeprom before write the defaul

    uint8_t i = 0;
    uint32_t totalADC1 = 0;
    uint32_t totalADC2 = 0;
    uint8_t cycles = 50;
    uint16_t rawReading1;
    uint16_t rawReading2;
    uint16_t processedReading1 = ADC_GetAPPS1Value();
    uint16_t processedReading2 = ADC_GetAPPS2Value();

    for(; i < cycles; ++i){
        rawReading1 = ADC_GetAPPS1Value();
        rawReading2 = ADC_GetAPPS2Value();
        LOWPASS_FILTER(rawReading1, processedReading1, appsAlpha);
        LOWPASS_FILTER(rawReading2, processedReading2, appsAlpha);
        totalADC1 += processedReading1;
        totalADC2 += processedReading2;
    }

    // get average
    uint16_t averageADC1 = totalADC1 / cycles;
    uint16_t averageADC2 = totalADC2 / cycles;

    // compare average to expected value and update if within reasonable range
    if (abs(averageADC1 - fullAPPS1_ADC) < APPS_ADC_DIFF_BUFF){
        fullAPPS1_ADC = averageADC1;
        EEPROM.put(apps1Full_address, fullAPPS1_ADC);
    } else {
        fullAPPS1_ADC = fullAPPS2_ADC;
        Faults_SetFault(FAULT_APPS_CALIBRATION_RESTING);
    }

    if (abs(averageADC2 - fullAPPS2_ADC) < APPS_ADC_DIFF_BUFF){
        fullAPPS2_ADC = averageADC2;
        EEPROM.put(apps2Full_address, fullAPPS2_ADC);
    } else {
        fullAPPS2_ADC = fullAPPS2_ADC;
        Faults_SetFault(FAULT_APPS_CALIBRATION_RESTING);
    }

#if APPS_CALIBRATION_DEBUG
    Serial.print("Full APPS1 ADC: ");
    Serial.println(fullAPPS1_ADC);
    Serial.print("APPS1 Average Full ADC : ");
    Serial.println(averageADC1);
    Serial.print("APPS1 Resting Comparison : ");
    Serial.println((averageADC1 == APPS1_FULL_PCT_ADC));

    Serial.print("Full APPS2 ADC: ");
    Serial.println(fullAPPS2_ADC);
    Serial.print("APPS2 Average Full ADC : ");
    Serial.println(averageADC2);
    Serial.print("APPS2 Full Comparison : ");
    Serial.println((averageADC2 == APPS2_FULL_PCT_ADC));
# endif


}

void APPS_Init() {
    appsData.appsReading1_Percentage = 0;
    appsData.appsReading2_Percentage = 0;

    appsData.appsReading1_Voltage = 0;
    appsData.appsReading2_Voltage = 0;

    appsData.apps1RawReading = 0;
    appsData.apps2RawReading = 0;

    appsAlpha = COMPUTE_ALPHA(40.0F);

    APPS_Calibrate_Rest();
}

void APPS_UpdateData(uint16_t rawReading1,
                     uint16_t rawReading2) { // changed uint16 from 32

    // Serial.print("Raw APPS1: ");
    // Serial.println(rawReading1);
    // Serial.print("Raw APPS2: ");
    // Serial.println(rawReading2);

    LOWPASS_FILTER(rawReading1, appsData.apps1RawReading, appsAlpha);
    LOWPASS_FILTER(rawReading2, appsData.apps2RawReading, appsAlpha);

    // Serial.print("\n\n\n\n\n");
    // Serial.print("Raw APPS1: ");
    // Serial.println(appsData.apps1RawReading);
    // Serial.print("Raw APPS2: ");
    // Serial.println(appsData.apps2RawReading);

    // if (appsData.appsReading2_Percentage < 0.0F) {
    //     appsData.appsReading2_Percentage = 0.0F;
    // } else if (appsData.appsReading2_Percentage > 1.0F) {
    //     appsData.appsReading2_Percentage = 1.0F;
    // }

    // after LOWPASS_FILTER
    appsData.appsReading1_Percentage =
        LINEAR_MAP(appsData.apps1RawReading, (float)restingAPPS1_ADC,
                   (float)fullAPPS1_ADC, 0.0F, 1.0F);

    appsData.appsReading2_Percentage =
        LINEAR_MAP(appsData.apps2RawReading, (float)restingAPPS2_ADC,
                   (float)fullAPPS2_ADC, 0.0F, 1.0F);

    /*========================== HELPER PCT CLAMP ==========================*/
    // clamp since LINEAR_MAP doesn't clamp
    // appsData.appsReading1_Percentage =
    //     CLAMP01(appsData.appsReading1_Percentage);
    // appsData.appsReading2_Percentage =
    //     CLAMP01(appsData.appsReading2_Percentage);

    // Convert ADC values to voltage
    appsData.appsReading1_Voltage =
        ADC_VALUE_TO_VOLTAGE(appsData.apps1RawReading);
    appsData.appsReading2_Voltage =
        ADC_VALUE_TO_VOLTAGE(appsData.apps2RawReading);

    /*========================== RAW VOLTAGE ==========================*/
    // Serial.print("APPS1 RAW Voltage: ");
    // Serial.println(appsData.appsReading1_Voltage);
    // Serial.print("APPS2 RAW Voltage: ");
    // Serial.println(appsData.appsReading2_Voltage);

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

    // Serial.print("APPS1 RAW Voltage: ");
    // Serial.println(appsData.appsReading1_Voltage);
    // Serial.print("APPS2 RAW Voltage: ");
    // Serial.println(appsData.appsReading2_Voltage);

    /*========================== 20 PCT LINEAR MAP ==========================*/
    // Moved this upwards to before the clamping of percentage
    // Map voltage to percentage of throttle travel, limiting to 0-1 range
    // appsData.appsReading1_Percentage =
    //     LINEAR_MAP(appsData.apps1RawReading, 0.0F, (float)APPS1_20PCT_ADC,
    //     0.0F, 1.0F);

    // appsData.appsReading2_Percentage =
    //     LINEAR_MAP(appsData.apps2RawReading, 0.0F, (float)APPS2_20PCT_ADC,
    //     0.0F, 1.0F);

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
        TickType_t elapsedTicks = now - appsLatestHealthyStateTimeAPPSRange;
        TickType_t elapsedMs = elapsedTicks * portTICK_PERIOD_MS;

        if (elapsedMs > APPS_FAULT_TIME_THRESHOLD_MS) {
#if DEBUG_FLAG
            Serial.println(elapsedMs);
            Serial.println("Setting APPS fault ELAPSED");
#endif
            Faults_SetFault(FAULT_APPS);
            return;
        }
    } else {
        appsLatestHealthyStateTimeAPPSRange = xTaskGetTickCount();
        Faults_ClearFault(FAULT_APPS);
    }

    if (difference > APPS_IMPLAUSABILITY_THRESHOLD) {
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsedTicks = now - appsLatestHealthyStateTimeAPPSDifference;
        TickType_t elapsedMs = elapsedTicks * portTICK_PERIOD_MS;

        if (elapsedMs > APPS_FAULT_TIME_THRESHOLD_MS) {
#if DEBUG_FLAG
            Serial.println(elapsedMs);
            Serial.println("Setting APPS fault ELAPSED");
#endif
            Faults_SetFault(FAULT_APPS);
            return;
        }
    } else {
#if DEBUG_FLAG
        Serial.println("Clearing fault in handle");
#endif
        appsLatestHealthyStateTimeAPPSDifference = xTaskGetTickCount();
        Faults_ClearFault(FAULT_APPS);
    }
}

static void checkAndHandlePlausibilityFault() {
    float BSEReading_Front = BSE_GetBSEReading()->bseFront_Reading;
    float BSEReading_Rear = BSE_GetBSEReading()->bseRear_Reading;

    float BSEReading = BSEReading_Front;
    if (BSEReading_Rear > BSEReading_Front) {
        BSEReading = BSEReading_Rear;
    }

#if DEBUG_FLAG
    Serial.print("BSE Reading: ");
    Serial.println(BSEReading);
#endif

    if (APPS_GetAPPSReading() > APPS_BSE_PLAUSABILITY_THROTTLE_THRESHOLD &&
        (BSEReading > APPS_BSE_PLAUSABILITY_BRAKE_THRESHOLD)) {
        Faults_SetFault(FAULT_APPS_BRAKE_PLAUSIBILITY);
    } else {
        if (APPS_GetAPPSReading() < APPS_BSE_PLAUSIBILITY_RESET_THRESHOLD) {
            Faults_ClearFault(FAULT_APPS_BRAKE_PLAUSIBILITY);
        }
    }
}
