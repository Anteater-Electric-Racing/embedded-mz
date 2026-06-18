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

// Active neutral point regen tuning
#define REGEN_BASE_NEUTRAL  0.15f   // zero-torque pedal position once engaged
#define REGEN_NEUTRAL_GAIN  0.10f   // extra neutral travel added at REGEN_RPM_MAX
#define REGEN_NEUTRAL_CAP   0.30f   // hard cap on neutral point (never > 30% pedal)
#define REGEN_RPM_MAX       6000.0f // expected max motor RPM (eRPM * 0.1)
#define REGEN_MIN_RPM       400.0f  // below this: no regen, full pedal = drive
#define REGEN_FADE_RPM      800.0f  // RPM window to fade the neutral zone in
#define REGEN_MAX_PCT       15.0f   // max regen as % of max motor current
#define REGEN_ALPHA         0.08f   // LPF alpha (~55ms ramp at 5ms loop rate)

static VehicleState vehicleState;
static DriveState driveState;
static TickType_t xLastWakeTime;

static bool enableRegen = false;
static float debugPedalDemand = 0.0f;
static float filteredRegen = 0.0f;

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

// Unified pedal map with active neutral point. Returns a SIGNED percentage:
//   >= 0 -> drive command  (relative current %, 0..CAPPED_MOTOR_TORQUE)
//   <  0 -> regen command  (relative brake current %, -REGEN_MAX_PCT..0)
// Both sides are percentages of max motor current, so they share units.
// The neutral crossover point shifts up with RPM to simulate engine friction:
//   low RPM  -> small neutral zone, easy to coast
//   high RPM -> wider neutral zone, backing off engages braking earlier
// Braking zone uses a quadratic curve for smooth tip-in (no drivetrain lash).
// Drive zone reuses the existing sigmoid via renormalized pedal position.
static float VCU_CalcUnifiedTorque(float pedal) {
    float rpm = fabsf(DTI_GetDTIData()->eRPM) * 0.1f;

    // Below the engagement speed there is no engine braking: the full pedal
    // range maps to drive torque, so launch and the base torque map stay intact.
    if (rpm < REGEN_MIN_RPM) {
        filteredRegen = 0.0f;
        return VCU_TorqueMap(pedal);
    }

    // Fade the neutral zone in over the engagement window so the bottom of the
    // pedal isn't suddenly stolen at the threshold (no torque step).
    float fade = (rpm - REGEN_MIN_RPM) / REGEN_FADE_RPM;
    if (fade > 1.0f) fade = 1.0f;

    float p_neutral =
        (REGEN_BASE_NEUTRAL + REGEN_NEUTRAL_GAIN * (rpm / REGEN_RPM_MAX)) * fade;
    if (p_neutral > REGEN_NEUTRAL_CAP) p_neutral = REGEN_NEUTRAL_CAP;

    if (pedal < p_neutral) {
        float brake_ratio = 1.0f - (pedal / p_neutral);
        if (brake_ratio > 1.0f) brake_ratio = 1.0f; // clamp negative pedal noise
        float target = -(brake_ratio * brake_ratio) * REGEN_MAX_PCT;
        LOWPASS_FILTER(target, filteredRegen, REGEN_ALPHA);
        return filteredRegen;
    } else {
        filteredRegen = 0.0f;
        float drive_ratio = (pedal - p_neutral) / (1.0f - p_neutral);
        return VCU_TorqueMap(drive_ratio);
    }
}

void threadVCU(void *pvParameters) {
    while (true) {
        vcu_last_run_tick = xTaskGetTickCount(); // update WDT tick
        float pedalAccel = APPS_GetAPPSReading();
        float pedalBrake = BSE_GetBSEAverage();
        Faults_HandleFaults();
        WSS_Update();

#if HIMAC_FLAG
        pedalAccel = debugPedalDemand;
#endif

        switch (vehicleState) {
        case STATE_PRECHARGING: /* default state */
            DTI_SendEnableCommand(false);

            // ENSURE never above 80kW limit (in DTI asw)
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
            if (PCC_PrechargeComplete()) {
                if (BSE_BrakesPressed()) {
                    if (RTM_ButtonState() && Faults_CheckAllClear()) {
                        Speaker_Play(); // Play Ready to Drive sound
                        vehicleState = STATE_DRIVING;
                    }
                } else {
                    RTM_ButtonReset();
                }
            } else {
                vehicleState = STATE_PRECHARGING;
            }
            // motorData.desiredTorque = 0.0F;
            break;
        case STATE_DRIVING: {
            // if (!HIMAC_FLAG || RTM_ButtonState() == false) {
            //     vehicleState = STATE_IDLE;
            // } else {

            if (RTM_ButtonState()) {
                DTI_SendEnableCommand(true);

                DTI_SetDCLimits(60.0, -2.0);
                DTI_SetACLimits(150.0, -20.0);

                // float batteryFactor =
                // VCU_Derate(BMS_GetOrionData()->highTemp); float motorFactor =
                // VCU_Derate(DTI_GetDTIData()->motorTemp); float inverterFactor
                // = VCU_Derate(DTI_GetDTIData()->controllerTemp);

                // // Get the Smallest Factor
                // float smallestFactor =
                //     min(batteryFactor, min(motorFactor, inverterFactor));

                if (BSE_BrakesPressed()) {
                    // brake pedal overrides: drop unified regen filter, handle
                    // separately so accel and brake commands never overlap
                    filteredRegen = 0.0f;
                    DTI_SendAccelCommand(0.0f);
                    if (enableRegen) {
                        DTI_SendBrakeCommand(pedalBrake);
                    }
                } else {
                    float signedTorque = VCU_CalcUnifiedTorque(
                        HIMAC_FLAG ? debugPedalDemand : pedalAccel);
                    if (signedTorque >= 0.0f) {
                        if (signedTorque > CAPPED_MOTOR_TORQUE)
                            signedTorque = CAPPED_MOTOR_TORQUE;
                        DTI_SendAccelCommand(signedTorque);
                    } else {
                        float regenPct = fabsf(signedTorque);
                        if (regenPct > REGEN_MAX_PCT)
                            regenPct = REGEN_MAX_PCT;
                        DTI_SendBrakeCommandRelative(regenPct);
                    }
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

void VCU_ClearFaultState() { vehicleState = STATE_IDLE; }

// void VCU_SetDebugPedalDemand(float pedalDemand) {
//     debugPedalDemand = constrain(pedalDemand, 0.0f, 1.0f);
// }

VehicleState VCU_GetState() { return vehicleState; }
