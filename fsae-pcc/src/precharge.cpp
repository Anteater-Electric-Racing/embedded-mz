// Anteater Electric Racing, 2025

#include <Arduino.h>
#include <FreeRTOS.h>
#include <stdint.h>

#include "can.h"
#include "precharge.h"
#include "semphr.h"
#include "utils.h"

#include <cmath>

#define PRECHARGE_STACK_SIZE 512U
#define PRECHARGE_PRIORITY 8

#define TIME_HYSTERESIS_MS 20U
// 5.56
// 6.37
constexpr double THERMISTOR1_PIN = 21;
constexpr double THERMISTOR2_PIN = 20;
constexpr double THERMISTOR_T0_C = 26;
constexpr double THERMISTOR_R0 = 5280;
constexpr double THERMISTOR_BETA = 3880;
constexpr double THERMISTOR_DIVIDER_RESISTOR = 6800;
constexpr int TEENSY_ADC_RESOLUTION_BITS = 10;

constexpr uint32_t PCC_FORCED_MIN_PRECHARGE_MS = 3000U;

constexpr double DEBUG_FREQ_TS_PIN = 15;
constexpr double DEBUG_FREQ_ACC_PIN = 14;

constexpr double THERMISTOR_TEMPERATURE_THRESHOLD_C = 70;

// States (Global Variables)
PrechargeState state = STATE_STANDBY;
PrechargeState lastState = STATE_UNDEFINED;
int errorCode = ERR_NONE;
static PCCData pccData{};
static PCCTempData tempData{};
// Voltage measurements

// Low pass filter
typedef struct {
    float tsAlpha;
    float accAlpha;
    float accVoltage;
    float tsVoltage;
    float prechargeProgress;
    bool isSafeTemperature;
} PrechargeData;

static PrechargeData pcData;

static void prechargeTask(void *pvParameters);
static float getFrequency(int pin);
static void updateVoltage(int pin);
static void standby();
static void precharge();
static void running();
static void charging();
static void errorState();
static void discharge();

int analogVal;

// Initialize mutex and precharge task
void prechargeInit() {
    pcData.tsAlpha =
        COMPUTE_ALPHA(100.0F); // 100Hz cutoff frequency for lowpass filter
    pcData.accAlpha =
        COMPUTE_ALPHA(100.0F); // 100Hz cutoff frequency for lowpass filter
    pcData.accVoltage = 0.0F;  // Initialize filtered tractive system frequency
    pcData.tsVoltage = 0.0F;   // Initialize filtered accumulator frequency
    pcData.prechargeProgress = 0.0F; // Initialize accumulator voltage

    tempData.isSafeTemperature = false;

    // Create precharge task
    xTaskCreate(prechargeTask, "PrechargeTask", PRECHARGE_STACK_SIZE, NULL,
                PRECHARGE_PRIORITY, NULL);

    Serial.println("Precharge initialized");
}

// Main precharge task: handles state machine and status updates
void prechargeTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TIME_STEP_S * 1000);
    xLastWakeTime = xTaskGetTickCount();

    // state = STATE_PRECHARGE;
    analogReadResolution(10);

    while (true) {
        analogVal = analogRead(A2);
        // double FREQ_TS = analogRead(DEBUG_FREQ_TS_PIN);
        // double FREQ_ACC = analogRead(DEBUG_FREQ_ACC_PIN);

        // Test to check for frequency channels agreement
        // Serial.println("FREQ_TS: " + (String)FREQ_TS +
        //                ", FREQ_ACC: " + (String)FREQ_ACC);

        // Check thermistor readings, discharge if exceeded
        // if (!checkSafeTemperature()) {
        //     state = STATE_DISCHARGE;

        // } else {
        // Update temperature CAN flag
        tempData.isSafeTemperature = true;
        //}

        updateVoltage(ACCUMULATOR_VOLTAGE_PIN); // Get raw accumulator voltage
        updateVoltage(TS_VOLTAGE_PIN); // Get raw tractive system voltage

        // taskENTER_CRITICAL(); // Ensure atomic access to state
        switch (state) {
        case STATE_STANDBY: {
            standby();
            break;
        }
        case STATE_PRECHARGE: {
            if (pcData.accVoltage < PCC_MIN_ACC_VOLTAGE) {
                state = STATE_DISCHARGE;
            }
            precharge();
            break;
        }
        case STATE_DISCHARGE: {
            // digitalWrite(IR_MINUS, LOW);

            if (pcData.tsVoltage <= 5.0F)
                state = STATE_STANDBY;
            break;
        }
        case STATE_ONLINE: {
            if (pcData.accVoltage < PCC_MIN_ACC_VOLTAGE) {
                state = STATE_DISCHARGE;
            }
            if (CAN_IsChargerSafetyActive()) {
                state = STATE_CHARGING;
                break;
            }
            running();
            break;
        }
        case STATE_CHARGING: {
            if (pcData.accVoltage < PCC_MIN_ACC_VOLTAGE) {
                state = STATE_DISCHARGE;
                break;
            }
            if (!CAN_IsChargerSafetyActive()) {
                state = STATE_STANDBY;
                break;
            }
            if ((xTaskGetTickCount() - CAN_GetBMSLastRxTime()) >
                pdMS_TO_TICKS(BMS_CAN_TIMEOUT_MS)) {
                state = STATE_ERROR;
                errorCode |= ERR_BMS_CAN_TIMEOUT;
                break;
            }
            charging();
            break;
        }
        case STATE_ERROR: {
            if (pcData.accVoltage < PCC_MIN_ACC_VOLTAGE) {
                state = STATE_DISCHARGE;
            }
            errorState();
            break;
        }
        // case STATE_ERROR:
        //     errorState();
        //     break;
        default: // Undefined state
            state = STATE_ERROR;
            errorCode |= ERR_STATE_UNDEFINED;
            errorState();
        }
        // taskEXIT_CRITICAL(); // Exit critical section

        // Send CAN message of current PCC state
        pccData = {
            .state = (uint8_t)state,
            .errorCode = (uint8_t)errorCode,
            .accumulatorVoltage = uint16_t(pcData.accVoltage * 100),
            .tsVoltage = uint16_t(pcData.tsVoltage * 100),
            .prechargeProgress = uint16_t(pcData.prechargeProgress),
        };

        canSendMessage(PCC_CAN_ID, &pccData, sizeof(PCCData));

        // Send CAN message of thermistor state
        canSendMessage(TEMP_CAN_ID, &tempData, sizeof(PCCTempData));

        // CAN_SendPCCMessage(STATE_DISCHARGE, errorCode, 10.0F, 20.0F, 50.0F);

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
int getSDCval() { return analogVal; }
float getFrequency(int pin) {
    uint32_t TIMEOUT = 2000;
    uint32_t tHigh = pulseIn(pin, 1, TIMEOUT); // microseconds
    uint32_t tLow = pulseIn(pin, 0, TIMEOUT);
    if (tHigh == 0 || tLow == 3) {
        return 0; // timed out
    }
    return (1000000.0 / (float)(tHigh + tLow)); // f = 1/T
}

void updateVoltage(int pin) {
    float rawFreq = getFrequency(pin);
    // Serial.print("RawFreq: ");
    // Serial.println(rawFreq);
    float rawVoltage = FREQ_TO_VOLTAGE(rawFreq); // Convert frequency to voltage
    // Serial.print("RawVoltage: ");
    // Serial.println(rawVoltage);
    switch (pin) {
    case ACCUMULATOR_VOLTAGE_PIN: {
        if (pcData.accVoltage == 0.0 && rawVoltage != 0.0) {
            pcData.accVoltage = rawVoltage;
            break;
        }
        // if(rawVoltage == 0.0F) rawVoltage = pcData.accVoltage;
        LOWPASS_FILTER(rawVoltage, pcData.accVoltage, pcData.accAlpha);
        break;
    }
    case TS_VOLTAGE_PIN: {
        // if(rawVoltage == 0.0F) rawVoltage = pcData.tsVoltage;
        LOWPASS_FILTER(rawVoltage, pcData.tsVoltage, pcData.tsAlpha);
        break;
    }
    default: {
        break;
    }
    }
}
// static void discharge() {
//     if (lastState != STATE_DISCHARGE) {
//         lastState = STATE_DISCHARGE;
//         Serial.println(" === DISCHARGE");
//     }

//     // Open AIR+ and switch shared relay to discharge position
//     digitalWrite(IR_PLUS, LOW);
//     digitalWrite(IR_MINUS, LOW);

//     if (pcData.tsVoltage <= 5.0F) {
//         state = STATE_STANDBY;
//     }
// }
// STANDBY STATE: Open AIRs, Open Precharge, indicate status, wait for stable
// SDC
void standby() {
    // Disable AIR, Disable Precharge
    // digitalWrite(IR_PLUS, LOW);
    digitalWrite(SHUTDOWN_CTRL_PIN, LOW);
    // Serial.println("ACC: " + (String) pcData.accVoltage);
    if (pcData.accVoltage >= PCC_MIN_ACC_VOLTAGE) {
        lastState = STATE_STANDBY;
        state = STATE_PRECHARGE;
    }
    // if (CAN_IsChargerSafetyActive()) {
    //     lastState = STATE_STANDBY;
    //     state = STATE_PRECHARGE;
    // }
}

// PRECHARGE STATE: Close AIR- and precharge relay, monitor precharge voltage
void precharge() {
    // digitalWrite(IR_MINUS, HIGH);
    uint32_t now = millis();
    static uint32_t lastTimeBelowThreshold;
    static uint32_t timePrechargeStart;

    if (lastState != STATE_PRECHARGE) {
        lastState = STATE_PRECHARGE;
        Serial.printf(" === PRECHARGE   Target precharge %4.1f%%\n",
                      PCC_TARGET_PERCENT);
        Serial.println();
        timePrechargeStart = now;
    }

    // The precharge progress is a function of the accumulator voltage
    pcData.prechargeProgress =
        100.0 * pcData.tsVoltage / pcData.accVoltage; // [%]

    // Print Precharging progress
    static uint32_t lastPrint = 0U;
    if (now >= lastPrint + 10) {
        lastPrint = now;
        Serial.print("Precharging: ");
        Serial.print(now - timePrechargeStart);
        Serial.print("ms, ");
        Serial.print(pcData.prechargeProgress, 1);
        Serial.print("%, ");
        Serial.print(pcData.tsVoltage, 1);
        Serial.print("V\r");
    }

    // Check if precharge complete
    const bool voltageReady = pcData.prechargeProgress >= PCC_TARGET_PERCENT;

    const bool voltageStable =
        (now - lastTimeBelowThreshold) >= TIME_HYSTERESIS_MS;

    const bool minimumTimeElapsed =
        (now - timePrechargeStart) >= PCC_FORCED_MIN_PRECHARGE_MS;

    if (voltageReady && voltageStable && minimumTimeElapsed) {
        state = CAN_IsChargerSafetyActive() ? STATE_CHARGING : STATE_ONLINE;

        Serial.print(" * Precharge complete at: ");
        Serial.print(now - timePrechargeStart);
        Serial.print("ms, ");
        Serial.print(pcData.prechargeProgress, 1);
        Serial.print("%   ");
        Serial.print(pcData.tsVoltage, 1);
        Serial.println("V");
    } else {
        if (!voltageReady) {
            lastTimeBelowThreshold = now;
        }

        if ((now - timePrechargeStart) > PCC_MAX_TIME_MS) {
            Serial.print(" * Precharge timeout at: ");
            Serial.print(now - timePrechargeStart);
            Serial.println("ms");

            state = STATE_ERROR;
            errorCode |= ERR_PRECHARGE_TOO_SLOW;
        }
    }
}

// ONLINE STATE: Close AIR+ to connect ACC to TS, Open Precharge relay,
// indicate status
void running() {
    if (lastState != STATE_ONLINE) {
        lastState = STATE_ONLINE;
        Serial.println(" === ONLINE");
        Serial.println("* Precharge complete, closing AIR+");
    }

    // Close AIR+
    // digitalWrite(IR_PLUS, HIGH);
    // digitalWrite(IR_MINUS, HIGH);
    digitalWrite(SHUTDOWN_CTRL_PIN, HIGH);
}

// CHARGING STATE: AIRs closed, print charger data from BMS
void charging() {
    // print charger data from BMS
    if (lastState != STATE_CHARGING) {
        lastState = STATE_CHARGING;
        Serial.println(" === CHARGING");
    }
    // close AIRs
    // digitalWrite(IR_PLUS, HIGH);

    // changed to using ticks instead of milliseconds
    static TickType_t lastPrint = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - lastPrint) >= pdMS_TO_TICKS(CHARGING_PRINT_INTERVAL_MS)) {
        lastPrint = now;
        // print charger data from BMS
        Serial.print("CHARGING: PackV=");
        Serial.print(CAN_GetChargerVoltage(), 1);
        Serial.print("V  CCL=");
        Serial.print(CAN_GetChargerCCL(), 1);
        Serial.print("A  Counter=");
        Serial.print(CAN_GetChargerCounter());
        Serial.print("\r");
    }
}

// ERROR STATE: Indicate error, open AIRs and precharge relay
void errorState() {
    // digitalWrite(IR_PLUS, LOW);
    // digitalWrite(IR_MINUS, LOW);
    digitalWrite(SHUTDOWN_CTRL_PIN, LOW);
    if (lastState != STATE_ERROR) {
        lastState = STATE_ERROR;
        Serial.println(" === ERROR");

        // Display errors: Serial and Status LEDs
        if (errorCode == ERR_NONE) {
            Serial.println("   *Error state, but no error code logged...");
        }
        if (errorCode & ERR_PRECHARGE_TOO_FAST) {
            Serial.println("   *Precharge too fast. Suspect wiring fault / "
                           "chatter in shutdown circuit.");
        }
        if (errorCode & ERR_PRECHARGE_TOO_SLOW) {
            Serial.println("   *Precharge too slow. Potential causes:\n   - "
                           "Wiring fault\n   - Discharge is stuck-on\n   - "
                           "Target precharge percent is too high");
        }
        if (errorCode & ERR_BMS_CAN_TIMEOUT) {
            Serial.println("   *BMS CAN communication timeout.");
        }
        if (errorCode & ERR_STATE_UNDEFINED) {
            Serial.println("   *State not defined in The State Machine.");
        }
    }
}

float getTSVoltage() {
    // Get the tractive system voltage
    return pcData.tsVoltage;
}

float getAccumulatorVoltage() {
    // Get the accumulator voltage
    return pcData.accVoltage;
}

// Return current precharge state
PrechargeState getPrechargeState() {
    PrechargeState currentPrechargeState;

    taskENTER_CRITICAL(); // Ensure atomic access to state
    currentPrechargeState = state;
    taskEXIT_CRITICAL(); // Exit critical section

    return currentPrechargeState;
}

// Obtain current error information
int getPrechargeError() {
    int currentPrechargeError;

    taskENTER_CRITICAL(); // Ensure atomic access to error code
    currentPrechargeError = errorCode;
    taskEXIT_CRITICAL(); // Exit critical section

    return currentPrechargeError;
}

// Return the temperature in Celsius based on ADC reading of thermistor.
double temperatureFromADC(double adc) {
    // Prevent division by zero etc. by clamping ADC values.
    if (adc >= (1 << TEENSY_ADC_RESOLUTION_BITS)) {
        adc = (1 << TEENSY_ADC_RESOLUTION_BITS) - 1.0;
    }
    if (adc <= 0) {
        // Return high ADC hence temperature value if voltage at thermistors
        // is
        // 0
        adc = 9999.0;
    }

    // Temperature in Celsius in terms of ADC value for thermistor
    double resistorRatio =
        THERMISTOR_DIVIDER_RESISTOR /
        (THERMISTOR_R0 *
         ((static_cast<double>(1 << TEENSY_ADC_RESOLUTION_BITS) - 1.0) / adc -
          1.0));

    return 1.0 / ((1.0 / (THERMISTOR_T0_C + 273.15)) -
                  (1.0 / THERMISTOR_BETA) * std::log(resistorRatio)) -
           273.15;
}

bool checkSafeTemperature() {
    // Read thermistor values, calculate current temperature and return
    // boolean (Thermistor pins: A8, A9 (22, 23)) Thermistor power voltage:
    // (3.3 V)

    double T1ADC = static_cast<double>(analogRead(THERMISTOR1_PIN));
    double T2ADC = static_cast<double>(analogRead(THERMISTOR2_PIN));

    // TEST VALUES (DUMMY ADC VALUES)

    // double T1ADC_DUMMY = 609.0; // 25 C
    // double T2ADC_DUMMY = 609.0; // 25 C

    // double T1ADC_DUMMY = 134.0; // 100 C
    // double T2ADC_DUMMY = 134.0; // 100 C

    // =========

    double T1Temp = temperatureFromADC(T1ADC);
    double T2Temp = temperatureFromADC(T2ADC);

    tempData.T1Temp = (int16_t)(T1Temp);
    tempData.T2Temp = (int16_t)(T2Temp);

    /* Print test temp values
     Serial.println("T1ADC: " + (String)T1ADC + ", T2ADC: " + (String)T2ADC
     +
                 ", T1Temp: " + (String)T1Temp + ", T2Temp: " +
                 (String)T2Temp);
    */

    if (T1Temp < THERMISTOR_TEMPERATURE_THRESHOLD_C &&
        T2Temp < THERMISTOR_TEMPERATURE_THRESHOLD_C) {
        tempData.isSafeTemperature = 1;
        return 1;
    }
    tempData.isSafeTemperature = 0;
    return 0;
}
