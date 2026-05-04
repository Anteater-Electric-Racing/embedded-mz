// Anteater Electric Racing, 2025

#define SPEED_CONTROL_ENABLED 0
#define SPEED_P_GAIN 0.01F // Proportional gain for speed control
#define SPEED_I_GAIN 0.1F  // Integral gain for speed control
#define LOW_VOLT_LIMIT 2.3F

constexpr float TEMP_START = 70.0f; // Temperature at which Derating Starts
constexpr float TEMP_MAX = 100.0f;  // Max Temperature, any Temperature greater
                                    // than this returns max derating factor

#include "vehicle/vcu.h"
#include "peripherals/can.h"
#include "peripherals/gpio.h"
#include "peripherals/wdt.h"

#include "utils/utils.h"

#include <arduino_freertos.h>

#include "vehicle/comms/bus.h"
#include "vehicle/comms/pcc.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/devices/apps.h"
#include "vehicle/devices/bse.h"
#include "vehicle/devices/dti.h"
#include "vehicle/devices/rtm.h"
#include "vehicle/faults.h"
#include <arduino_freertos.h>

template <typename T> T constrain(T val, T minVal, T maxVal) {
    if (val < minVal)
        return minVal;
    if (val > maxVal)
        return maxVal;
    return val;
}

static VehicleState vehicleState;
static DriveState driveState;
static TickType_t xLastWakeTime;

static bool enableRegen = false;
static float debugPedalDemand = 0.0f;

// EMRAX 228 MV motor parameters from DTI HV-550 config tool
constexpr float DTI_EMRAX_POLE_PAIRS = 10.0f;
constexpr float DTI_EMRAX_LAMBDA_PM_WB = 0.071002f; // 71.002 mWb
constexpr float DTI_EMRAX_LD_H = 0.00014480f;       // 144.80 µH
constexpr float DTI_EMRAX_LQ_H = 0.00014480f;       // 144.80 µH
constexpr float MIN_FLUX_LINKAGE_WB = 0.001f;
constexpr float MAX_AC_DRIVE_CURRENT_A = 150.0f;

static float VCU_TorqueToCurrent(float torqueNm) {
    // Use actual measured d-axis and q-axis currents from packet 0x23
    // (PKT_DTI5).
    float idActual = DTI_GetDTIData()->focId;
    float lambdaEff =
        DTI_EMRAX_LAMBDA_PM_WB + (DTI_EMRAX_LD_H - DTI_EMRAX_LQ_H) * idActual;

    if (lambdaEff >= 0.0f && lambdaEff < MIN_FLUX_LINKAGE_WB) {
        lambdaEff = MIN_FLUX_LINKAGE_WB;
    } else if (lambdaEff < 0.0f && lambdaEff > -MIN_FLUX_LINKAGE_WB) {
        lambdaEff = -MIN_FLUX_LINKAGE_WB;
    }

    float iqFromTorque =
        (2.0f * torqueNm) / (3.0f * DTI_EMRAX_POLE_PAIRS * lambdaEff);
    float currentMagnitude =
        sqrtf(idActual * idActual + iqFromTorque * iqFromTorque);

    return currentMagnitude;
}

// Define 3 Presets (Steepness k, Midpoint x0)
// Map 0: Rain (High precision, late power)
// Map 1: Endurance (Balanced, predictable)
// Map 2: Autocross  (More linear + induce level shift)

const float k_vals[] PROGMEM = {10.0f, 9.0f, 12.0f};
const float x0_vals[] PROGMEM = {0.7f, 0.425f, 0.375f};

static float k = 0.0f, x0 = 0.0f, low_limit = 0.0f, high_limit = 0.0f;

void VCU_Init() {
    vehicleState = STATE_PRECHARGING; // DEFAULT TO PRECHARGE
    enableRegen = false;

    driveState.controlMode = TORQUE;
    driveState.driveStrategy = OPEN_LOOP;
    DTI_LinkControlMode(&driveState.controlMode);

    k = k_vals[ACTIVE_MAP];
    x0 = x0_vals[ACTIVE_MAP];

    low_limit = 1.0f / (1.0f + expf(-k * (0.0f - x0)));
    high_limit = 1.0f / (1.0f + expf(-k * (1.0f - x0)));
}

void threadVCU(void *pvParameters) {
    while (true) {
        vcu_last_run_tick = xTaskGetTickCount(); // update WDT tick
        float pedalAccel = APPS_GetAPPSReading();
        float pedalBrake = BSE_GetBSEAverage();
        Faults_HandleFaults();

#if HIMAC_FLAG
        pedalAccel = debugPedalDemand;
#endif

        switch (vehicleState) {
        case STATE_PRECHARGING: /* default state */
            DTI_SendEnableCommand(false);
            DTI_SetDCLimits(60.0, -2.0);
            DTI_SetACLimits(150.0, -20.0);
            if (PCC_PrechargeComplete()) {
                vehicleState = STATE_IDLE;
            }
            break;
        case STATE_IDLE:
            DTI_SendEnableCommand(false);
            //  transition to IDLE
            //  TODO Update brake light threshold if we only want to move when
            //  mech brakes are engaged
            if (BSE_BrakesPressed()) {
                if (RTM_ButtonState() && Faults_CheckAllClear()) {
                    vehicleState = STATE_DRIVING;
                }
            } else {
                RTM_ButtonReset();
            }
            // motorData.desiredTorque = 0.0F;
            break;
        case STATE_DRIVING: {
            // if (!HIMAC_FLAG || RTM_ButtonState() == false) {
            //     vehicleState = STATE_IDLE;
            // } else {
            DTI_SendEnableCommand(true);

            float targetTorque = 0.0f;
            if (HIMAC_FLAG) {
                targetTorque = VCU_TorqueMap(debugPedalDemand);
            } else {
                targetTorque = VCU_TorqueMap(pedalAccel);
            }
            float batteryFactor = VCU_Derate(BMS_GetOrionData()->highTemp);
            float motorFactor = VCU_Derate(DTI_GetDTIData()->motorTemp);
            float inverterFactor = VCU_Derate(DTI_GetDTIData()->controllerTemp);

            // Get the Smallest Factor
            float smallestFactor =
                min(batteryFactor, min(motorFactor, inverterFactor));

            float dcCurrentLimit = 60.0f * smallestFactor;
            float acCurrentLimit = MAX_AC_DRIVE_CURRENT_A * smallestFactor;
            DTI_SetDCLimits(dcCurrentLimit, -2.0);
            DTI_SetACLimits(acCurrentLimit, -20.0);

            float deratedTorque = targetTorque * smallestFactor;
            float targetCurrent = VCU_TorqueToCurrent(deratedTorque);
            targetCurrent = constrain(targetCurrent, 0.0f, acCurrentLimit);

            // Convert absolute current (A) to relative percentage (0-100%)
            float relativeCurrentPercent =
                (targetCurrent / acCurrentLimit) * 100.0f;
            DTI_SendAccelCommand(relativeCurrentPercent);

            // Serial.println(targetTorque * smallestFactor);
            if (enableRegen && BSE_BrakesPressed()) {
                DTI_SendBrakeCommand(pedalBrake);
            }
        }

        break;
        case STATE_FAULT:
            // DTI_SendEnableCommand(false);
            if (Faults_CheckAllClear()) {
                VCU_ClearFaultState();
            }
            break;
        default:
            break;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

float VCU_Derate(float temperature) {
    float factor = 1.0f;
    float min_factor = 0.2f;
    temperature = constrain(temperature, TEMP_START, TEMP_MAX);
    // Piecewise Linear Derating
    factor = 1.0f - (1.0f - min_factor) *
                        ((temperature - TEMP_START) / (TEMP_MAX - TEMP_START));
    return factor;
}

// TODO switch to LUT for all applicable strategies
float VCU_TorqueMap(float pedal) {

    float target = 0.0f;
    switch (driveState.driveStrategy) {
    case OPEN_LOOP:
        // Raw Sigmoid curve
        {
            float raw = 1.0f / (1.0f + expf(-k * (pedal - x0)));
            float normalized_ratio =
                (raw - low_limit) / (high_limit - low_limit);
            target = (normalized_ratio * CAPPED_MOTOR_TORQUE);
            break;
        }
    case TRACTION_CTRL: {
        /* TC implementation */
    } break;
    case LAUNCH_CTRL: {
        /* LC implemnetation */
    } break;
    default: {
        break;
    }
    }
    return target;
}
void VCU_SetFaultState() { vehicleState = STATE_FAULT; }

void VCU_SetState(VehicleState state) { vehicleState = state; }

void VCU_ForceIdleState() { RTM_ButtonReset(); }

void VCU_ClearFaultState() { vehicleState = STATE_DRIVING; }

void VCU_SetDebugPedalDemand(float pedalDemand) {
    debugPedalDemand = constrain(pedalDemand, 0.0f, 1.0f);
}

VehicleState VCU_GetState() { return vehicleState; }
