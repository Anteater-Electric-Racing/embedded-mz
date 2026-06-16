// Anteater Electric Racing, 2025

#pragma once

/*
HIGH PRIORITY
--> simplify state machine using DTI, write docs of how to config dti for mz,
testing docs
--> tssi bypass (using IMDCAN readings + 3 sec delay to then trigger normal
operation)
--> imd thershold increase
--> LookupTable for throttle curve

        TODO: better serial monitor (python script with pySerial)
    (constantly running) needs all necessary fields

        Cooling PID control loop based on MCU and Motor Temp

        Derating of torque based on temp.

        Power limiting based on BMS current seen (if accurate)

        IMD data check (1 msg read, HV mesage not read) -- IMD error code 512 is
    read until status is cleared

        clean up code

    TODO Fixes:
    BIG MZ Change - INTERUPT BASED CAN Testing:
*/
constexpr int fault_address = 0;

#define SERIALMONITOR_FLAG 1
#define DEBUG_FLAG 0
#define HIMAC_FLAG 0
#define WSS_FLAG 0
#define BMS_FLAG 0 // REMOVE
#define IMD_FLAG 1
#define APPS_DEBUG 1
#define BSE_DEBUG 0
#define PRECHARGE_DEBUG 1

#define ACTIVE_MAP 1

#define HIGH 1
#define LOW 0

#define THREAD_MAIN_STACK_SIZE 128
#define THREAD_MAIN_PRIORITY 1
#define THREAD_MOTOR_STACK_SIZE 256
#define THREAD_MOTOR_PRIORITY 4
#define THREAD_CAN_TELEMETRY_STACK_SIZE 512
#define THREAD_CAN_TELEMETRY_PRIORITY 1
#define THREAD_ADC_STACK_SIZE 128
#define THREAD_ADC_PRIORITY 8
#define THREAD_CP_STACK_SIZE 128
#define THREAD_CP_PRIORITY 6
#define THREAD_WDT_STACK_SIZE 128
#define THREAD_WDT_PRIORITY 9

/**
 *
 *
 */

#define WHEEL_SPEED_1_PIN 2
#define WHEEL_SPEED_2_PIN 3
#define rtm_PIN 36
#define BRAKE_LIGHT_PIN 9

#define LOGIC_LEVEL_V 3.3F
#define TIME_STEP 0.001F // 1ms time step

#define ADC_AVERAGING 1
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)
#define TICKTYPE_FREQUENCY 1

#define ADC_VOLTAGE_DIVIDER 1.88F
#define ADC_VOLTAGE_DIVIDER1 1.88F
#define ADC_VOLTAGE_DIVIDER2 1.36F

#define ADC_VALUE_TO_VOLTAGE(x, divider)                                       \
    ((x) * (LOGIC_LEVEL_V * (divider) / ADC_MAX_VALUE))

#define APPS_FAULT_PERCENT_MIN .1F
#define APPS_FAULT_PERCENT_MAX .9F

#define APPS1_VOLTAGE_LEVEL 3.3F
#define APPS2_VOLTAGE_LEVEL 3.3F

// idt this is the play
#define APPS_RANGE_MIN_PERCENT .15F
#define APPS_RANGE_MAX_PERCENT .85F

// #define APPS_3V3_MIN 1.45  //(APPS1_VOLTAGE_LEVEL * APPS_RANGE_MIN_PERCENT)
// #define APPS_3V3_MAX 2.00F //(APPS1_VOLTAGE_LEVEL * APPS_RANGE_MAX_PERCENT)

// #define APPS_3V3_INV_MIN 1.76F //(APPS2_VOLTAGE_LEVEL *
// APPS_RANGE_MIN_PERCENT) #define APPS_3V3_INV_MAX 1.25F //(APPS2_VOLTAGE_LEVEL
// * APPS_RANGE_MAX_PERCENT)

extern float APPS1_REST_ADC;
extern float APPS2_REST_ADC;

#define APPS1_FULL_PCT_ADC 460.21F
#define APPS2_FULL_PCT_ADC 2480.51F

// // Clamp helper
// #define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
// #define CLAMP01(x) CLAMP((x), 0.0F, 1.0F)

// // Convert raw ADC -> commanded percent using only 0-20% physical range
// #define APPS_ADC_TO_CMD_PERCENT(adc, rest_adc, adc_20)
//     CLAMP01(((float)(adc) - (float)(rest_adc)) /
//             ((float)(adc_20) - (float)(rest_adc)))

// noise adjustment
#define APPS_3V3_MIN                                                           \
    ADC_VALUE_TO_VOLTAGE(APPS1_REST_ADC, ADC_VOLTAGE_DIVIDER1) - 0.15F
#define APPS_3V3_MAX                                                           \
    ADC_VALUE_TO_VOLTAGE(APPS1_FULL_PCT_ADC, ADC_VOLTAGE_DIVIDER1) + 0.15F

#define APPS_3V3_INV_MIN                                                       \
    ADC_VALUE_TO_VOLTAGE(APPS2_REST_ADC, ADC_VOLTAGE_DIVIDER2) + 0.15F
#define APPS_3V3_INV_MAX                                                       \
    ADC_VALUE_TO_VOLTAGE(APPS2_FULL_PCT_ADC, ADC_VOLTAGE_DIVIDER2) - 0.15F

// go back to idle faults
#define APPS_3V3_FAULT_MIN                                                     \
    ADC_VALUE_TO_VOLTAGE(APPS1_REST_ADC, ADC_VOLTAGE_DIVIDER1) - 0.3F
#define APPS_3V3_FAULT_MAX                                                     \
    ADC_VALUE_TO_VOLTAGE(APPS1_FULL_PCT_ADC, ADC_VOLTAGE_DIVIDER1) + 0.3F

#define APPS_3V3_INV_FAULT_MIN                                                 \
    ADC_VALUE_TO_VOLTAGE(APPS2_REST_ADC, ADC_VOLTAGE_DIVIDER2) + 0.3F
#define APPS_3V3_INV_FAULT_MAX                                                 \
    ADC_VALUE_TO_VOLTAGE(APPS2_FULL_PCT_ADC, ADC_VOLTAGE_DIVIDER2) - 0.3F
#define APPS_FAULT_TIME_THRESHOLD_MS 100

#define APPS_IMPLAUSABILITY_THRESHOLD 0.10F            // 10%
#define APPS_BSE_PLAUSABILITY_THROTTLE_THRESHOLD 0.25F // 25% as per rules
#define APPS_BSE_PLAUSABILITY_BRAKE_THRESHOLD                                  \
    0.50 // TODO: change back to PSI200    // IN VOLTS --> use PSI value here
#define APPS_BSE_PLAUSIBILITY_RESET_THRESHOLD 0.05F // 5%

extern float BSE_MIN_PSI;
#define BSE_MAX_PSI 1000.0F
extern float BSE_MIN_V;
#define BSE_MAX_V 4.5F
#define BSE_VOLTAGE_TO_PSI(x)                                                  \
    (BSE_MAX_PSI / (BSE_MAX_V - BSE_MIN_V)) * (x - BSE_MIN_V)

// ALL in volts rn
#define BRAKE_LIGHT_THRESHOLD 4.0F // PSI
#define BRAKE_LIGHT_AVG_THRESHOLD 142.5F
#define BSE_LOWER_THRESHOLD 0.25F
#define BSE_UPPER_THRESHOLD 4.5F
#define BSE_IMPLAUSABILITY_THRESHOLD 0.1F

#define BSE_FAULT_TIME_THRESHOLD_MS 100

#define BSE_CUTOFF_HZ 40.0F

#define CAN_FAULT_TIME_THRESHOLD_MS 100

#define MOTOR_MAX_TORQUE 220.0F // TODO: Update with real value //used to be 220
#define CAPPED_MOTOR_TORQUE 80.0F
#define MAX_TORQUE_STEP_UP_PCT 0F
#define MAX_TORQUE_STEP_DOWN_PCT 1.0F
#define TORQUE_SHIFT_OFFSET 5.0F

#define BATTERY_MAX_CURRENT_A 140.0F // TO CHANGE
#define BATTERY_MAX_REGEN_A 140.0F   // TO CHANGE

#define COMPUTE_ALPHA(CUTOFF_HZ)                                               \
    (1.0F / (1.0F + (1.0F / (2.0F * M_PI * CUTOFF_HZ)) / TIME_STEP))

#define LOWPASS_FILTER(NEW, OLD, ALPHA) OLD = ALPHA * NEW + (1.0F - ALPHA) * OLD

#define LINEAR_MAP(x, in_min, in_max, out_min, out_max)                        \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define DTI_16_SCALE 0.1F
#define DTI_32_SCALE 0.01F

// TODO depreciate the following
#define CHANGE_ENDIANESS_16(x) (((x & 0xFF00) >> 8) | ((x & 0x00FF) << 8))
#define CHANGE_ENDIANESS_32(x)                                                 \
    (((x & 0x000000FF) << 24) | ((x & 0x0000FF00) << 8) |                      \
     ((x & 0x00FF0000) >> 8) | ((x & 0xFF000000) >> 24))

#define SWAP_16(x) __builtin_bswap16(x)
#define SWAP_32(x) __builtin_bswap32(x)
#define SWAP_64(x) __builtin_bswap64(x)

#define MOTOR_DIRECTION_STANDBY 0
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_BACKWARD 2
#define MOTOR_DIRECTION_ERROR 3

#define MAX_REGEN_TORQUE -9.0F // TODO: test with higher value regen
#define REGEN_BIAS 1           // Scale 0-1 of max regen torque

#define SHOCK_TRAVEL_MAX_MM 76.2f // 3 inches
