// Anteater Electric Racing, 2025

#define SPEED_CONTROL_ENABLED 0
#define SPEED_P_GAIN 0.01F // Proportional gain for speed control
#define SPEED_I_GAIN 0.1F  // Integral gain for speed control

#define LOW_VOLT_LIMIT 2.3F

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
#include "vehicle/vcu.h"

static VehicleState vehicleState;
static DriveState driveState;
static TickType_t xLastWakeTime;

static bool enableRegen = false;

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
        case STATE_DRIVING:
            if (RTM_ButtonState() == false) {
                vehicleState = STATE_IDLE;
            } else {
                DTI_SendEnableCommand(true);

                float targetTorque = VCU_TorqueMap(pedalAccel);

                // derate current before sending

                DTI_SendAccelCommand(targetTorque);
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

void VCU_ForceIdleState() {
    vehicleState = STATE_FAULT;
    RTM_ButtonReset();
}

void VCU_ClearFaultState() { vehicleState = STATE_DRIVING; }

VehicleState VCU_GetState() { return vehicleState; }
