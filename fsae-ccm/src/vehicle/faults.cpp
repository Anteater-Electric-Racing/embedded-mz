// Anteater Electric Racing, 2025

#define FAULT_OVER_CURRENT_MASK (0x1)
#define FAULT_UNDER_VOLTAGE_MASK (0x1 << 1)
#define FAULT_OVER_TEMP_MASK (0x1 << 2)
#define FAULT_APPS_MASK (0x1 << 3)
#define FAULT_BSE_MASK (0x1 << 4)
#define FAULT_BPPS_MASK (0x1 << 5)
#define FAULT_APPS_BRAKE_PLAUSIBILITY_MASK (0x1 << 6)
#define FAULT_CAN_MASK (0x1 << 7)
#define LOW_BATTERY_VOLTAGE_MASK (0x1 << 8)

#include "vehicle/faults.h"

#include <Arduino.h>

static uint32_t faultBitMap;

void Faults_Init() { faultBitMap = 0; }

void Faults_SetFault(FaultType fault) {
    switch (fault) {
    case FAULT_NONE: {
        break;
    }
    case FAULT_OVER_CURRENT: {
        faultBitMap |= FAULT_OVER_CURRENT_MASK;
        break;
    }
    case FAULT_UNDER_VOLTAGE: {
        faultBitMap |= FAULT_UNDER_VOLTAGE_MASK;
        break;
    }
    case FAULT_OVER_TEMP: {
        faultBitMap |= FAULT_OVER_TEMP_MASK;
        break;
    }
    case FAULT_APPS: {
#if DEBUG_FLAG
        Serial.println("Setting APPS fault");
#endif
        faultBitMap |= FAULT_APPS_MASK;
        break;
    }
    case FAULT_BSE: {
#if DEBUG_FLAG
        Serial.println("Setting BSE fault");
#endif
        faultBitMap |= FAULT_BSE_MASK;
        break;
    }
    case FAULT_BPPS: {
        faultBitMap |= FAULT_BPPS_MASK;
        break;
    }
    case FAULT_APPS_BRAKE_PLAUSIBILITY: {
#if DEBUG_FLAG
        Serial.println("Setting APPS Plausibility fault");
#endif
        faultBitMap |= FAULT_APPS_BRAKE_PLAUSIBILITY_MASK;
        break;
    }
    case FAULT_CAN: {
#if DEBUG_FLAG
        Serial.println("Setting CAN Timeout Fault");
#endif
        faultBitMap |= FAULT_CAN_MASK;
        break;
    }
    case LOW_BATTERY_VOLTAGE_FAULT: {
        faultBitMap |= LOW_BATTERY_VOLTAGE_MASK;
        break;
    }
    default: {
        break;
    }
    }
}

void Faults_ClearFault(FaultType fault) {
    switch (fault) {
    case FAULT_NONE: {
        break;
    }
    case FAULT_OVER_CURRENT: {
        faultBitMap &= ~FAULT_OVER_CURRENT_MASK;
        break;
    }
    case FAULT_UNDER_VOLTAGE: {
        faultBitMap &= ~FAULT_UNDER_VOLTAGE_MASK;
        break;
    }
    case FAULT_OVER_TEMP: {
        faultBitMap &= ~FAULT_OVER_TEMP_MASK;
        break;
    }
    case FAULT_APPS: {
#if DEBUG_FLAG
        Serial.println("Clearing APPS fault");
#endif
        faultBitMap &= ~FAULT_APPS_MASK;
        break;
    }
    case FAULT_BSE: {
        faultBitMap &= ~FAULT_BSE_MASK;
        break;
    }
    case FAULT_BPPS: {
        faultBitMap &= ~FAULT_BPPS_MASK;
        break;
    }
    case FAULT_APPS_BRAKE_PLAUSIBILITY: {
#if DEBUG_FLAG
        Serial.println("Clearing APPS BSE plausibility fault");
#endif
        faultBitMap &= ~FAULT_APPS_BRAKE_PLAUSIBILITY_MASK;
        break;
    }
    case FAULT_CAN: {
#if DEBUG_FLAG
        Serial.println("Clearing CAN Timeout Fault");
#endif
        faultBitMap &= ~FAULT_CAN_MASK;
        break;
    }
    case LOW_BATTERY_VOLTAGE_FAULT: {
        faultBitMap &= ~LOW_BATTERY_VOLTAGE_MASK;
        break;
    }
    default: {
        break;
    }
    }
}

// currently having all faults being handled the same but leaving room for
// future customization
void Faults_HandleFaults() {

#if DEBUG_FLAG
    Serial.print("Fault bitmap: ");
    Serial.println(faultBitMap);
#endif

    if (faultBitMap == 0) {
        // #if DEBUG_FLAG
        //         Serial.println("Clearing all faults in handle faults");
        // #endif

        //         Motor_ClearFaultState();
        return;
    }
    if (faultBitMap & FAULT_OVER_CURRENT_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_UNDER_VOLTAGE_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_OVER_TEMP_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_APPS_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_BSE_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_BPPS_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_APPS_BRAKE_PLAUSIBILITY_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & FAULT_CAN_MASK) {
        VCU_SetFaultState();
    }
    if (faultBitMap & LOW_BATTERY_VOLTAGE_MASK) {
        VCU_SetFaultState();
    }
}

// for telemetry
uint32_t Faults_GetFaults() { return faultBitMap; }

bool Faults_CheckAllClear() { return faultBitMap == 0; }
