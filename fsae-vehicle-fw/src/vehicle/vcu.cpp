// Anteater Electric Racing, 2025

#define SPEED_CONTROL_ENABLED 0
#define SPEED_P_GAIN 0.01F // Proportional gain for speed control
#define SPEED_I_GAIN 0.1F  // Integral gain for speed control

#define LOW_VOLT_LIMIT 2.3F

#include "peripherals/can.h"
#include "peripherals/gpio.h"
#include "utils/utils.h"
#include <arduino_freertos.h>

#include "vehicle/comms/can_polls.h"
#include "vehicle/comms/pcc_receive.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/devices/apps.h"
#include "vehicle/devices/bse.h"
#include "vehicle/devices/rtm_button.h"
#include "vehicle/faults.h"
#include "vehicle/vcu.h"

typedef struct {
    MotorState state;
    float desiredTorque; // Torque demand in Nm;
} MotorData;

static MotorData motorData;
static TickType_t xLastWakeTime;

// Define 3 Presets (Steepness k, Midpoint x0)
// Map 0: Rain (High precision, late power)
// Map 1: Endurance (Balanced, predictable)
// Map 2: Autocross  (More linear + induce level shift)

/*

intial default k = 9.0, x0 = 0.35
karan/gabenote: gets to speed too qucik (nears max at 60% pedal travel)
3/11 change (tbd): x0 = 0.425 (nears max at 75% pedal travel)

*/

/**
 * create struct of motorFSM inputs
 *
 *
 * Pseudocode for DTI Motor FSM
 * start in state precharge --> once precharged, idle mode is when RTD signal
 * (DTI is disabled) want to map the RTD button on the car to RTD signal on DTI
 * then the torque command can be sent periodically (simlar to before), where
 * the update Motor function is called based on torque?
 *
 * is updateMotor still neeeded?
 * do a thread call threadMotor. No need for motor_update motor anymore. in the
 * packet to send. RTD signal just set to high when pressed, off when pressed
 * again (toggle)
 *
 * remove updateMotor
 *
 * add Provision for Regen (in driving state, how is regen applied)
 * helper function for APPStoTorqueLookup
 * helper function for BSEtoRegen
 * blending should be done in fsm. --> threadMotor is just fsm
 */

// define this in header, constexpr can be used
const float k_vals[] PROGMEM = {10.0f, 9.0f, 12.0f};
const float x0_vals[] PROGMEM = {0.7f, 0.425f, 0.375f};

float k = 0.0f, x0 = 0.0f, low_limit = 0.0f, high_limit = 0.0f;
float targetTorque = 0.0F;

void Motor_Init() {
    motorData.state = MOTOR_STATE_PRECHARGING; // DEFAULT TO PRECHARGE
    motorData.desiredTorque = 0.0F;            // No torque demand at start

    k = k_vals[ACTIVE_MAP];
    x0 = x0_vals[ACTIVE_MAP];

    low_limit = 1.0f / (1.0f + expf(-k * (0.0f - x0)));
    high_limit = 1.0f / (1.0f + expf(-k * (1.0f - x0)));
}

void threadMotor(void *pvParameters) {
    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

#if !HIMAC_FLAG
void Motor_UpdateMotor(float torqueDemand) {

    Faults_HandleFaults();
    RTMButton_Update(GPIO_Read(RTM_BUTTON_PIN));
    switch (motorData.state) {
    case MOTOR_STATE_PRECHARGING: /* default state */
        if ((PCC_GetData()->prechargeProgress >= 85 &&
             PCC_GetData()->state == 3)) {
            motorData.state = MOTOR_STATE_IDLE;
        }
        motorData.desiredTorque = 0.0F;
        break;
    case MOTOR_STATE_IDLE:
        // transition to IDLE
        // TODO Update brake light threshold if we only want to move when
        // mech brakes are engaged
        if (BSE_GetBSEReading()->bseFront_Reading >= BRAKE_LIGHT_THRESHOLD &&
            BSE_GetBSEReading()->bseRear_Reading >= BRAKE_LIGHT_THRESHOLD) {
            if (RTMButton_GetState() && Faults_CheckAllClear()) {
                motorData.state = MOTOR_STATE_DRIVING;
            }
        } else {
            RTMButton_Reset();
        }
        motorData.desiredTorque = 0.0F;
        break;
    case MOTOR_STATE_DRIVING:
        if (RTMButton_GetState()) {

            motorData.desiredTorque = torqueDemand;

        } else {
            motorData.state = MOTOR_STATE_IDLE;
            torqueDemand = 0;
        }

        break;
    case MOTOR_STATE_FAULT:
        // RTMButton_Reset(); // force set RTM OFF
        motorData.desiredTorque = 0.0F;
        if (Faults_CheckAllClear()) {
            Motor_ClearFaultState();
        }
        break;
    default:
        break;
    }
}

#endif

#if HIMAC_FLAG
void Motor_UpdateMotor(float torqueDemand, bool enablePrecharge,
                       bool enablePower, bool enableRun, bool enableRegen,
                       bool enableStandby) {

    uint8_t prechargeState = PCC_GetData()->state;
    uint16_t prechargeProg = PCC_GetData()->prechargeProgress;
    // off --> standby --> precharge --> run --> fault -->standy
    // no kl15 then off
    switch (motorData.state) {
    case MOTOR_STATE_OFF: {
        if (enableStandby) {
            // # if HIMAC_FLAG
            //     Serial.println("Standby Mode");
            // #endif
            motorData.state = MOTOR_STATE_STANDBY;
        }
        motorData.desiredTorque = 0.0F;
        break;
    }
    // kl15 on and ready to go into precharge
    case MOTOR_STATE_STANDBY: {
        if (enablePrecharge) {

            motorData.state = MOTOR_STATE_PRECHARGING;
        }
        motorData.desiredTorque = 0.0F;
        break;
    }
    // HV switch on (PCC CAN message)
    case MOTOR_STATE_PRECHARGING: {
        if ((prechargeProg >= 94 && prechargeState == 3) || enablePower) {
            // # if HIMAC_FLAG
            //     Serial.println("Precharge finished");
            // # endif
            motorData.state = MOTOR_STATE_IDLE;
        } else if (enableStandby) {
            motorData.state = MOTOR_STATE_STANDBY;
        }

        motorData.desiredTorque = 0.0F;
        break;
    }
    // PCC CAN message finished
    case MOTOR_STATE_IDLE: {
        if (enableRun) {
            motorData.state = MOTOR_STATE_DRIVING;
        }
        motorData.desiredTorque = 0.0F;
        break;
    }
    // Ready to drive button pressed
    case MOTOR_STATE_DRIVING: {
        if (enableRun) {

            if (enableRegen && torqueDemand <= 0.0F &&
                MCU_GetMCU1Data()->motorDirection == MOTOR_DIRECTION_FORWARD) {
                // If regen is enabled and the torque demand is zero, we
                // need to set the torque demand to 0 to prevent the motor
                // from applying torque in the wrong direction
                motorData.desiredTorque = MAX_REGEN_TORQUE * REGEN_BIAS;
            } else {
                motorData.desiredTorque = torqueDemand;
            }

        } else {
            motorData.state = MOTOR_STATE_IDLE;
            torqueDemand = 0;
        }
        break;
    }
    // Any fault error occurs
    case MOTOR_STATE_FAULT: {

        if (enableStandby) {
            motorData.state = MOTOR_STATE_STANDBY;
        } else if (enableRun) {
            Motor_ClearFaultState();
        }
        motorData.desiredTorque = 0.0F;
        break;
    }
    default: {
        break;
    }
    }
}
#endif

float torqueMap(float pedal) {
    // Raw Sigmoid curve
    float raw = 1.0f / (1.0f + expf(-k * (pedal - x0)));
    float normalized_ratio = (raw - low_limit) / (high_limit - low_limit);
    float target = (normalized_ratio * CAPPED_MOTOR_TORQUE);
    return target;
}

float Motor_GetTorqueDemand() { return motorData.desiredTorque; }

void Motor_SetFaultState() { motorData.state = MOTOR_STATE_FAULT; }

void Motor_ClearToIdleFault() {
    motorData.state = MOTOR_STATE_FAULT;
    RTMButton_Reset();
}

void Motor_ClearFaultState() { motorData.state = MOTOR_STATE_DRIVING; }

MotorState Motor_GetState() { return motorData.state; }

float Motor_TargetTorque() { return targetTorque; }
