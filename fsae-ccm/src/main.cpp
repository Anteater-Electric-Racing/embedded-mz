// Anteater Electric Racing, 2025

#include <Arduino.h>
#include <arduino_freertos.h>

#include "peripherals/adc.h"
#include "peripherals/can.h"
#include "peripherals/gpio.h"
#include "peripherals/wdt.h"

#include "peripherals/gpio.h"
#include "vehicle/comms/bus.h"
#include "vehicle/comms/pcc.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/controls/bypass.h"
#include "vehicle/devices/apps.h"
#include "vehicle/devices/bse.h"
#include "vehicle/devices/linpots.h"
#include "vehicle/devices/rtm.h"
#include "vehicle/devices/speaker.h"
#include "vehicle/devices/thermal.h"
#include "vehicle/devices/wss.h"
#include "vehicle/faults.h"
#include "vehicle/vcu.h"

#include "utils/utils.h"
#include <iostream>
#include <unistd.h>

#define TORQUE_STEP 1
#define TORQUE_MAX_NM 120 // Maximum torque demand in Nm

static TickType_t xLastWakeTime;

void threadMain(void *pvParameters);

void setup() { // runs once on bootup

    Serial.begin(9600);

    ADC_Init();
    Bus_Init();
    CAN_Init();
    APPS_Init();
    Shock_Init();
    BSE_Init();
    Faults_Init();
    Telemetry_Init();
    VCU_Init();
    GPIO_Init();
    PCC_Init();
    thermal_Init();
    Bypass_Init();
    GPIO_Init();
    WSS_Init();
    WDT_Init();
    Speaker_Init();

    xTaskCreate(threadADC, "threadADC", THREAD_ADC_STACK_SIZE, NULL,
                THREAD_ADC_PRIORITY, NULL);
    xTaskCreate(threadBus, "threadBus", THREAD_CP_STACK_SIZE, NULL,
                THREAD_CP_PRIORITY, NULL);
    xTaskCreate(threadVCU, "threadVCU", THREAD_MOTOR_STACK_SIZE, NULL,
                THREAD_MOTOR_PRIORITY, NULL);
    xTaskCreate(threadTelemetry, "threadTelemetry",
                THREAD_CAN_TELEMETRY_STACK_SIZE, NULL,
                THREAD_CAN_TELEMETRY_PRIORITY, NULL);

    xTaskCreate(threadMain, "threadMain", THREAD_MAIN_STACK_SIZE, NULL,
                THREAD_MAIN_PRIORITY, NULL);
    xTaskCreate(threadWDT, "threadWDT", THREAD_WDT_STACK_SIZE, NULL,
                THREAD_WDT_PRIORITY, NULL);
    vTaskStartScheduler();
}

void threadMain(void *pvParameters) {
    xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time

#if HIMAC_FLAG
    float torqueDemand = 0;
    VCU_SetDebugPedalDemand(0.0f);

    bool enableRegen = false;
#endif
    while (true) {
        // WSS_Update();
        main_last_run_tick = xTaskGetTickCount(); // update WDT tick
        thermal_regulate();
        /*============ LOW PRIORITY GPIO UPDATES ============*/
        digitalWrite(13, HIGH); // orange led on teensy
        Bypass_TSSI();

        RTM_ButtonUpdate(digitalRead(rtm_PIN));

        // thermal_regulate(); //still need to tune parameters

        if (BSE_GetBSEAverage() > BRAKE_LIGHT_AVG_THRESHOLD) {
            digitalWrite(BRAKE_LIGHT_PIN, HIGH);
        } else {
            digitalWrite(BRAKE_LIGHT_PIN, LOW);
        }

#if APPS_DEBUG
        Serial.print("APPS1 %: ");
        Serial.print(APPS_GetAPPSReading1());
        Serial.print(" | ");
        Serial.print("APPS2 %: ");
        Serial.print(APPS_GetAPPSReading2());
        Serial.print(" | ");
        Serial.print("Diff: ");
        Serial.print(abs(APPS_GetAPPSReading1() - APPS_GetAPPSReading2()));
        Serial.print(" | ");
        Serial.print("Fault bitmap: ");
        Serial.print(Faults_GetFaults(), arduino::BIN);
        // Serial.print("\r");

#endif

#if BSE_DEBUG
        Serial.print(" BSE Reading 1 ");
        Serial.print(BSE_GetBSEReading()->bseRear_Reading);
        Serial.print(" | ");
        Serial.print("BSE Reading 2 ");
        Serial.print(BSE_GetBSEReading()->bseFront_Reading);
        Serial.print(" | ");
        Serial.print("BSE Avg ");
        Serial.print(BSE_GetBSEAverage());
        Serial.print(" | ");
        Serial.print("BSE Threshold: ");
        Serial.print(BRAKE_LIGHT_AVG_THRESHOLD);
        Serial.print(" | ");
        Serial.print("Fault bitmap: ");
        Serial.println(Faults_GetFaults(), arduino::BIN);

#endif

#if WSS_FLAG
        Serial.print("W1 RPM: ");
        Serial.print(WSS_GetRPM1());
        Serial.print(" | W1 MPH: ");
        Serial.print(WSS_GetSpeed1_MPH());

        Serial.print(" | W2 RPM: ");
        Serial.print(WSS_GetRPM2());
        Serial.print(" | W2 MPH: ");
        Serial.print(WSS_GetSpeed2_MPH());
        Serial.print(" | ");
        Serial.print("W3 RPM: ");
        Serial.print(WSS_GetRPM3());
        Serial.print(" | W3 MPH: ");
        Serial.print(WSS_GetSpeed3_MPH());

        Serial.print(" | W4 RPM: ");
        Serial.print(WSS_GetRPM4());
        Serial.print(" | W4 MPH: ");
        Serial.print(WSS_GetSpeed4_MPH());
#endif

#if SERIALMONITOR_FLAG

        Serial.print("State: ");
        Serial.print(VCU_GetState());
        Serial.print(" | RTM: ");
        Serial.print(digitalRead(rtm_PIN));
        Serial.print(" | Fault bitmap: ");
        Serial.print(Faults_GetFaults(), arduino::BIN);
        Serial.print(" | InvCurr: ");
        Serial.print(DTI_GetDTIData()->acCurrent);
        Serial.print("A | DriveEn: ");
        Serial.print(DTI_GetDTIData()->driveEnabled);
        Serial.print(" | Mode: ");
        Serial.print(DTI_GetDTIData()->controlMode);
        Serial.print(" | RPM: ");
        Serial.print(DTI_GetDTIData()->eRPM);
        Serial.print(" | ThrottleIn: ");
        Serial.print(APPS_GetAPPSReading());
        Serial.print("% | TorqueOut: ");
        Serial.print(DTI_GetDTIData()->targetIq);
        Serial.print(" | DutyCycle: ");
        Serial.print(DTI_GetDTIData()->dutyCycle, 2);
        Serial.print(" | BatTemp: ");
        Serial.print(BMS_GetOrionData()->highTemp);
        Serial.print("C | MotorTemp: ");
        Serial.print(DTI_GetDTIData()->motorTemp);
        Serial.print("C | InvTemp: ");
        Serial.print(DTI_GetDTIData()->controllerTemp);
        Serial.print("C");

        // Serial.print("\r");
        //  IMPLEMENT BETTER SERIAL PROCESSING(
        //      TEENSY does not support ANSI escape codes)
#endif
#if PRECHARGE_DEBUG
        Serial.print(" | PCC_State: ");
        Serial.print(PCC_GetData()->state);
        Serial.print(" | Prog: ");
        Serial.print(PCC_GetData()->prechargeProgress);
        Serial.print(" | ACC_V: ");
        Serial.print(PCC_GetData()->accumulatorVoltage);
        Serial.print(" | TS_V: ");
        Serial.print(PCC_GetData()->tsVoltage);
#endif
#if BMS_FLAG
        // --- NEW: Orion BMS 2 Telemetry ---
        // Orion BMS Telemetry
        Serial.print(" | BMS Volt: ");
        Serial.print(BMS_GetOrionData()->packVoltage);
        Serial.print("V | SOC: ");
        Serial.print(BMS_GetOrionData()->soc);
        Serial.print("% | Current: ");
        Serial.print(BMS_GetOrionData()->packCurrent);

        // // Thermal and Limits (From Message 0x6B1)
        Serial.print(" | HiTemp: ");
        Serial.print(BMS_GetOrionData()->highTemp);
        Serial.print("C | DCL: ");
        Serial.print(BMS_GetOrionData()->dischargeLimit);
        Serial.print("A");

        // Cell Health (From Message 0x6B2)
        Serial.print(" | AvgCell: ");
        Serial.print(BMS_GetOrionData()->avgCellVolt,
                     4); // 4 decimal places for precision
        Serial.print("V | HiCell: ");
        Serial.print(BMS_GetOrionData()->highCellVolt, 4);

#endif

#if IMD_FLAG

        Serial.print(" IMD_HV: ");
        Serial.print(IMD_GetInfo()->hv_voltage);
        Serial.print(" | ");
        Serial.print("IMD_Resistance: ");
        Serial.print(IMD_GetInfo()->resistance);
        Serial.print(" | ");
        Serial.print("IMD_Status: ");
        Serial.print(IMD_GetInfo()->status);
        Serial.print(" | ");
        Serial.print("IMD_Fault: ");
        Serial.print(IMD_GetInfo()->isolation_fault);
        Serial.print(" | ");

#endif

        Serial.print("\r");
#if HIMAC_FLAG

        /*
         * Read user input from Serial to control torque demand.
         * 'w' or 'W' to increase torque demand,(apply throttle)
         * 's' or 'S' to decrease torque demand, (let go throttle)
         *
         * '2' key = Precharging
         * '3' key = IDLE
         * '4' key = DRIVE
         * 'f' is fault state
         *
         * ' ' (space) to stop all torque. (reset w/s to 0)
         *
         * The torque demand is limited between 0 and TORQUE_MAX_NM.
         *
         * Telemetry to show in 1 line :
         * battery current, invt side current, contorl mode
         * motor speed RPM, throttle input and output read by invt
         *
         * For copilot, look at the structs in dti.h or bus.h to see what
         * data is avaliable
         *
         * temperature(s)
         */

        if (Serial.available()) {
            char input = Serial.read();

            switch (input) {
            case '2': {
                VCU_SetState(STATE_PRECHARGING);
                torqueDemand = 0;
                VCU_SetDebugPedalDemand(0.0f);
                break;
            }
            case '3': {
                VCU_SetState(STATE_IDLE);
                torqueDemand = 0;
                VCU_SetDebugPedalDemand(0.0f);
                break;
            }
            case '4': {
                VCU_SetState(STATE_DRIVING);
                break;
            }
            case 'w': // Increase torque demand
            case 'W': {
                if (torqueDemand < TORQUE_MAX_NM) {
                    torqueDemand += TORQUE_STEP; // Increment torque demand
                }
                VCU_SetDebugPedalDemand(torqueDemand / TORQUE_MAX_NM);
                break;
            }
            case 's': // Decrease torque demand
            case 'S': {
                if (torqueDemand > 0) {
                    torqueDemand -= TORQUE_STEP; // Decrement torque demand
                }
                VCU_SetDebugPedalDemand(torqueDemand / TORQUE_MAX_NM);
                break;
            }
            case ' ': { // Stop all torque
                torqueDemand = 0;
                VCU_SetDebugPedalDemand(0.0f);

                break;
            }
            case 'f': // Fault state
            case 'F': {
                VCU_SetFaultState(); // Set VCU to fault state
                break;
            }
            default:
                break;
            }
        }

        // Telemetry: battery current, invt side current, control mode, motor
        // speed RPM, throttle input and output, temperatures
        Serial.print("State: ");
        Serial.print(VCU_GetState());
        Serial.print(" | InvCurr: ");
        Serial.print(DTI_GetDTIData()->acCurrent);
        Serial.print("A | DriveEn: ");
        Serial.print(DTI_GetDTIData()->driveEnabled);
        Serial.print(" | Mode: ");
        Serial.print(DTI_GetDTIData()->controlMode);
        Serial.print(" | RPM: ");
        Serial.print(DTI_GetDTIData()->eRPM);
        Serial.print(" | ThrottleIn: ");
        Serial.print((torqueDemand / TORQUE_MAX_NM));
        Serial.print("% | ThrottleOut: ");
        Serial.print(DTI_GetDTIData()->targetIq);
        Serial.print(" | DutyCycle: ");
        Serial.print(DTI_GetDTIData()->dutyCycle, 2);
        Serial.print(" | BatTemp: ");
        Serial.print(BMS_GetOrionData()->highTemp);
        Serial.print("C | MotorTemp: ");
        Serial.print(DTI_GetDTIData()->motorTemp);
        Serial.print("C | InvTemp: ");
        Serial.print(DTI_GetDTIData()->controllerTemp);
        Serial.print("C");

        Serial.print("\r");

#endif
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // Delay for 100ms
    }
}

void loop() {}
