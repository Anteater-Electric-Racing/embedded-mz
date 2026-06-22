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
#include "vehicle/devices/speaker.h"
#include "vehicle/devices/wss.h"
#include "vehicle/faults.h"
#include "vehicle/vcu.h"
#include <arduino_freertos.h>

// template <typename T> T constrain(T val, T minVal, T maxVal) {
//     if (val < minVal)
//         return minVal;
//     if (val > maxVal)
//         return maxVal;
//     return val;
// }

static VehicleState vehicleState;
static DriveState driveState;
static TickType_t xLastWakeTime;

static bool enableRegen = false;
static float debugPedalDemand = 0.0f;

// Define 3 Presets (Steepness k, Midpoint x0)
// Map 0: Rain (High precision, late power)
// Map 1: Endurance (Balanced, predictable)
// Map 2: Autocross  (More linear + induce level shift)

const float k_vals[] PROGMEM = {10.0f, 9.0f, 12.0f};
const float x0_vals[] PROGMEM = {0.7f, 0.425f, 0.375f};

static float k = 0.0f, x0 = 0.0f, low_limit = 0.0f, high_limit = 0.0f;

void VCU_Init() {
    vehicleState = STATE_IDLE; // DEFAULT TO PRECHARGE
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
        float targetTorque = 0.0f;
        Faults_HandleFaults();
        WSS_Update();

#if HIMAC_FLAG
        pedalAccel = debugPedalDemand;
#endif

        switch (vehicleState) {
        case STATE_PRECHARGING: /* default state */
            vehicleState = STATE_IDLE;
            DTI_SendEnableCommand(false);

            // ENSURE never above 80kW limit (in DTI asw)
            DTI_SetDCLimits(AC_MAX, -2.0);
            DTI_SetACLimits(AC_MAX, -AC_MAX_R);
            if (PCC_PrechargeComplete()) {
                vehicleState = STATE_IDLE;
            }
            break;
        case STATE_IDLE:

            if (PCC_PrechargeComplete()) {

                vehicleState = STATE_DRIVING;
            }
            //  transition to IDLE
            //  TODO Update brake light threshold if we only want to move when
            //  mech brakes are engaged
            if (BSE_BrakesPressed()) {
                if (RTM_ButtonState() && Faults_CheckAllClear()) {

                    // assume rtm button gets sent, stays 1
                    Speaker_Play(); // Play Ready to Drive sound
                }
            } else {
                RTM_ButtonReset();
                DTI_SendEnableCommand(false);
            }
            // motorData.desiredTorque = 0.0F;
            break;
        case STATE_DRIVING: {
            // if (!HIMAC_FLAG || RTM_ButtonState() == false) {
            //     vehicleState = STATE_IDLE;
            // } else {

            if (RTM_ButtonState()) {

                DTI_SendEnableCommand(true);
                if (HIMAC_FLAG) {
                    targetTorque = VCU_TorqueMap(debugPedalDemand);
                } else {
                    targetTorque = VCU_TorqueMap(pedalAccel);
                }

                // float batteryFactor =
                // VCU_Derate(BMS_GetOrionData()->highTemp); float
                // motorFactor = VCU_Derate(DTI_GetDTIData()->motorTemp);
                // float inverterFactor =
                // VCU_Derate(DTI_GetDTIData()->controllerTemp);

                // // Get the Smallest Factor
                // float smallestFactor =
                //     min(batteryFactor, min(motorFactor, inverterFactor));

                DTI_SetDCLimits(AC_MAX, -2.0);
                DTI_SetACLimits(AC_MAX, -AC_MAX_R);

                DTI_SendAccelCommand(targetTorque);

                // Serial.println(targetTorque * smallestFactor);
                if (enableRegen && BSE_BrakesPressed()) {
                    DTI_SendBrakeCommand(pedalBrake);
                }

            } else {
                vehicleState = STATE_IDLE;
                targetTorque = 0;
            }

        } break;

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

// float VCU_Derate(float temperature) {
//     float factor = 1.0f;
//     float min_factor = 0.2f;
//     temperature = constrain(temperature, TEMP_START, TEMP_MAX);
//     // Piecewise Linear Derating
//     factor = 1.0f - (1.0f - min_factor) *
//                         ((temperature - TEMP_START) / (TEMP_MAX -
//                         TEMP_START));
//     return factor;
// }

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

void VCU_ForceFaultIdleState() { RTM_ButtonReset(); }

void VCU_ClearFaultState() { vehicleState = STATE_DRIVING; }

// void VCU_SetDebugPedalDemand(float pedalDemand) {
//     debugPedalDemand = constrain(pedalDemand, 0.0f, 1.0f);
// }

VehicleState VCU_GetState() { return vehicleState; }
