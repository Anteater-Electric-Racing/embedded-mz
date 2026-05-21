// Testing Launch Control System by Rishi and Anoop

#include <Arduino.h>
#include <algorithm>
#include <arduino_freertos.h>

#include "peripherals/can.h"

#include "launch.h"
#include "../comms/telemetry.h"
#include "dti.h"
#include "utils/PID.h"
#include "bse.h"
#include "apps.h"
#include "vehicle/faults.h"

//#include "../utils/pid.h"


static PID slipPIDConfig;
static float slipTarget = 0.07f;   // 7% slip
static float minTorque = 0.0f;
static float wheelRadius = 0.5f; // in meters, adjust based on actual wheel radius
static TickType_t xLastWakeTime;
static float torqueDemand = 0.0f;
/*static VCU1 vcu1 = {.BMS_Main_Relay_Cmd = 1, 
    .VehicleState = 1, .GearLeverPos_Sts = 3, .AC_Control_Cmd = 1, 
    .BMS_Aux_Relay_Cmd = 1, .KeyPosition = 2}; */

static LaunchState launchControlState = LAUNCH_STATE_OFF;

void LaunchControl_Init()
{
    pidReset(&slipPIDConfig);
    if(pidConfig(&slipPIDConfig, INTEGRAL_MAX, INTEGRAL_MIN) != 0) {
        // Handle PID configuration error
    }
    // Initialize PID control parameters and *maybe* set default state at false for driver choice
}

void threadLaunchControl(void *pvParameters)
{
    while (true) {
        //vcu1 = {0};
        switch (launchControlState) {
            case LAUNCH_STATE_ON:{
                float wheelSpeedFL = 0.0f; // placeholder
                float wheelSpeedFR = 0.0f; // placeholder
            }
                float motorSpeed = DTI_GetDTIData()->eRPM * rpmConversion;

                float realTorque = Motor_GetState()->torqueCmd; // Current torque command from motor controller
                //This is assuming we are obtaining wheel speeds in rad/s

                float controlledSpeed = motorSpeed * wheelRadius; // Obtain the higher speed of the wheels connected to Powertrain for safety precaution
                float freeSpeed = std::min(wheelSpeedFL * wheelRadius, wheelSpeedFR * wheelRadius);
                if(freeSpeed == 0.0f) // Free roaming wheels (front two) and take the lower speed of these for safety precaution
                {
                    freeSpeed = 0.001f; // To avoid division by zero
                }
                float slipRatio = (controlledSpeed - freeSpeed) / freeSpeed; //Slip Ratio equation provided by Vik

                // if(slipRatio < 0.05f || slipRatio > 0.15f)
                // {        //Doubt this is needed, as PID should be able to handle this
                // }
                

                float maxTorqueNm = Telemetry_GetData()->maxMotorTorque; // Just to understand Max Torque so we don't exceed this
                //float dt = 0.01f; // Time step for PID calculation, adjusted for 100Hz update rate Pranav: shouldnt be needed using tick counts instead
                
                // PID Control for Slip Ratio
                float correction = computePID(&slipPIDConfig, slipTarget, slipRatio, KP, KI, KD); // Proportional control for slip ratio
                // Currently unsure how this will be integrated, as the correction should reduce torque when slip is high and increase when low, but unsure what values would come out
                torqueDemand = realTorque + correction; // Reduce torque based on slip ratio correction


                // limit torque demand to be within allowable limits(no less then 0 and no more the nmaxToqrue which I believe is 260 according to Utils.h)
                if(torqueDemand > maxTorqueNm)
                {
                    torqueDemand = maxTorqueNm;
                }
                else if(torqueDemand < minTorque)
                {
                    torqueDemand = minTorque;
                }

                if (std::max(BSE_GetBSEReading()->bseFront_Reading, BSE_GetBSEReading()->bseRear_Reading) > 50.0f ||
                    APPS_GetAPPSReading() < 1.0f ||
                    Motor_GetState() != MOTOR_STATE_DRIVING) {
                    // If brake is pressed, disable launch control
                    pidReset(&slipPIDConfig);
                    launchControlState = LAUNCH_STATE_OFF;
                } else {
                    // Continue applying torque demand
                    Launch_setTorqueDemand();
                }
                break;

            case LAUNCH_STATE_OFF:
                // Implement launch control logic here
                if ((std::max(BSE_GetBSEReading()->bseFront_PSI, BSE_GetBSEReading()->bseRear_PSI) > 50.0f) && (MCU_GetMCU1Data()->motorSpeed == 0.0f) && Motor_GetState() == MOTOR_STATE_DRIVING) {     
                    launchControlState = LAUNCH_STATE_ON;   //If Car isn't moving and brake is pressed, enable launch control to active
                }
                break;
            case LAUNCH_STATE_FAULT:
                // Handle fault state
                pidReset(&slipPIDConfig);
                torqueDemand = 0.0f;
                Launch_setTorqueDemand();
                launchControlState = LAUNCH_STATE_OFF;
                //Faults_SetFault(FAULT_LAUNCH_CONTROL);
                break;
            }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void Launch_setTorqueDemand() {
    vcu1.VCU_TorqueReq = (uint8_t) ((fabsf(torqueDemand) / MOTOR_MAX_TORQUE) * 100); // Torque demand in percentage (0-99.6) 350Nm
    vcu1.VCU_MotorMode = torqueDemand >= 0 ? 1 : 2; 
    uint64_t vcu1_msg;
    memcpy(&vcu1_msg, &vcu1, sizeof(vcu1_msg));
    CAN_Send(mVCU1_ID, vcu1_msg);
}

LaunchState Launch_getState() {
    return launchControlState;
}