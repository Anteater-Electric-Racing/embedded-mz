// Anteater Electric Racing, 2025

#include "bus.h"
#include "arduino_freertos.h"
#include "imd.h"
#include "pcc.h"
#include "peripherals/can.h"
#include "peripherals/wdt.h"
#include "telemetry.h"
#include "utils/utils.h"

static TickType_t xLastWakeTime;
static uint32_t rx_id;
static uint64_t rx_data;

static dtiData1 dtiData;
static dtiData2 dtiExtra;

static OrionBMSData bmsData;
static IMDData imdData;

static TickType_t canLatestHealthyStateTime = 0;
static uint32_t canAgeMs = 0;

void Bus_Init() {

    dtiData = {.controlMode = 0,
               // 1: CONTROL_MODE_SPEED
               // 2: CONTROL_MODE_CURRENT
               // 3: CONTROL_MODE_CURRENT_BRAKE
               // 4: CONTROL_MODE_POS
               // 7: CONTROL_MODE_NONE
               // 0, 5, 6: NOT USED
               .targetLq = 0,
               .motorPosition = 0, // in degrees
               .isMotorStill = 0,  // in still position or not
               .eRPM = 0,      // eRPM = motor RPM * number of motor pole pairs
               .dutyCycle = 0, // controller duty cycle
               .inputVoltage = 0, // dcVoltage
               .acCurrent = 0,    // AC motor current (sign is regen or running)
               .dcCurrent = 0,    // AC motor current (sign is regen or running)
               .controllerTemp = 0, // temp of inverter semiconductors
               .motorTemp = 0,      // temp of motor measured by inverter
               .faultCode = 0,      // all inverter faults, add to faultMAP TODO
               .focLd = 0,          // foc alg Ld
               .focLq = 0,          // foc alg lq.
               .driveEnabled = 0,   // RTM toggle.
               .maxAC_Current = 0,
               .avMaxAC_Current = 0,
               .minAC_Current = 0,
               .avMinAC_Current = 0,
               .maxDC_Current = 0,
               .avMaxDC_Current = 0,
               .minDC_Current = 0,
               .avMinDC_Current = 0};

    dtiExtra = {.throttleInput = 0, // throttle straight to inverter
                .brakeInput = 0,
                .digitalIn1 = false,  // digital input
                .digitalIn2 = false,  // digital input
                .digitalIn3 = false,  // digital input
                .digitalIn4 = false,  // digital input
                .digitalOut1 = false, // digital output
                .digitalOut2 = false, // digital output
                .digitalOut3 = false, // digital output
                .digitalOut4 = false, // digital output // byte 4
                .capTempLimitActive = false,
                .dcTempLimitActive = false,
                .driveEnableLimitActive = false,
                .IGBTaccelLimitActive = false,
                .IGBTtempLimitActive = false,
                .inputVoltageLimitActive = false,
                .motorAccelTempLimitActive = false,
                .motorTempLimitActive = false,
                .RPMminLimitActive = false,
                .RPMmaxLimitActive = false,
                .powerLimitActive = false,
                .zero1 = false,
                .zero2 = false,
                .zero3 = false,
                .zero4 = false,
                .zero5 = false,
                .reserved = 0,
                .CANmapVersion = 0};

    bmsData = {.packCurrent = 0.0F,
               .packVoltage = 0.0F,
               .soc = 0.0F,
               .relayState = 0,
               .dischargeLimit = 0.0F,
               .chargeLimit = 0.0F,
               .highTemp = 25,
               .lowTemp = 25};

    imdData = {.resistance = 0.0F, // kOhm
               .hv_voltage = 0.0F, // Volts
               .status = 0,        // Raw flags
               .isolation_fault = false};

    // Initialize the motor thread
}

void threadBus(void *pvParameters) {
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        can_last_run_tick = xTaskGetTickCount(); // update WDT tick

        // Read the CAN messages
        if (CAN_Receive(&rx_id, &rx_data)) {
            canLatestHealthyStateTime = xTaskGetTickCount();
            // update healthy state time on successful CAN
        }
        canAgeMs = (can_last_run_tick - canLatestHealthyStateTime) *
                   portTICK_PERIOD_MS;
        if (canAgeMs > CAN_FAULT_TIME_THRESHOLD_MS) { // 100 ms
            Faults_SetFault(FAULT_CAN);
        } else {
            Faults_ClearFault(FAULT_CAN);
        }

        // TODO ADD SHIFT  >> by 8 here will ONLY work for DTI.. maybe not.
        // distinguish case for all in same loop??
        switch ((rx_id >> 8)) {
        case PKT_1_ID: {
            PKT_DTI1 dti1 = {0};
            memcpy(&dti1, &rx_data, sizeof(dti1));

            taskENTER_CRITICAL(); // Enter critical section
            dtiData.controlMode = dti1.controlMode,
            dtiData.targetLq =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti1.targetLq)) *
                DTI_16_SCALE;
            dtiData.motorPosition =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti1.motorPosition)) *
                DTI_16_SCALE; // in degrees
            dtiData.isMotorStill =
                dti1.isMotorStill; // in still position or not
            taskEXIT_CRITICAL();   // Exit critical section
            break;
        }

        case PKT_2_ID: {
            PKT_DTI2 dti2 = {0};
            memcpy(&dti2, &rx_data, sizeof(dti2));

            taskENTER_CRITICAL(); // Enter critical section
            dtiData.eRPM = (float)((int32_t)CHANGE_ENDIANESS_32(dti2.eRPM));
            dtiData.dutyCycle =
                (float)((int16_t)SWAP_16(dti2.dutyCycle)) * DTI_16_SCALE;
            dtiData.inputVoltage =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti2.inputVoltage));
            taskEXIT_CRITICAL(); // Exit critical section
            break;
        }

        case PKT_3_ID: {
            PKT_DTI3 dti3 = {0};
            memcpy(&dti3, &rx_data, sizeof(dti3));
            taskENTER_CRITICAL(); // Enter critical section
            dtiData.acCurrent =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti3.acCurrent)) *
                DTI_16_SCALE;
            dtiData.dcCurrent =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti3.dcCurrent)) *
                DTI_16_SCALE;

            taskEXIT_CRITICAL(); // Exit critical section
            break;
        }
        case PKT_4_ID: {
            PKT_DTI4 dti4 = {0};
            memcpy(&dti4, &rx_data, sizeof(dti4));
            taskENTER_CRITICAL();
            dtiData.controllerTemp =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti4.controllerTemp)) *
                DTI_16_SCALE;
            dtiData.motorTemp =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti4.motorTemp)) *
                DTI_16_SCALE;
            dtiData.faultCode = dti4.faultCode, taskEXIT_CRITICAL();
            break;
        }
        case PKT_5_ID: {
            PKT_DTI5 dti5 = {0};
            memcpy(&dti5, &rx_data, sizeof(dti5));
            taskENTER_CRITICAL();
            dtiData.focLd = (float)((int32_t)CHANGE_ENDIANESS_32(dti5.focLd)) *
                            DTI_32_SCALE;
            dtiData.focLq = (float)((int32_t)CHANGE_ENDIANESS_32(dti5.focLq)) *
                            DTI_32_SCALE;
            taskEXIT_CRITICAL();
            break;
        }
        case PKT_6_ID: {
            PKT_DTI6 dti6 = {0};
            memcpy(&dti6, &rx_data, sizeof(dti6));
            taskENTER_CRITICAL();
            dtiData.driveEnabled = dti6.driveEnabled;

            dtiExtra.throttleInput = (float)dti6.throttleInput;
            dtiExtra.brakeInput = (float)dti6.brakeInput;
            dtiExtra.digitalIn1 = dti6.digitalIn1;   // digital input
            dtiExtra.digitalIn2 = dti6.digitalIn2;   // digital input
            dtiExtra.digitalIn3 = dti6.digitalIn3;   // digital input
            dtiExtra.digitalIn4 = dti6.digitalIn4;   // digital input
            dtiExtra.digitalOut1 = dti6.digitalOut1; // digital output
            dtiExtra.digitalOut2 = dti6.digitalOut2; // digital output
            dtiExtra.digitalOut3 = dti6.digitalOut3; // digital output
            dtiExtra.digitalOut4 = dti6.digitalOut4;
            dtiExtra.capTempLimitActive = dti6.capTempLimitActive;
            dtiExtra.dcTempLimitActive = dti6.dcTempLimitActive;
            dtiExtra.driveEnableLimitActive = dti6.driveEnableLimitActive;
            dtiExtra.IGBTaccelLimitActive = dti6.IGBTaccelLimitActive;
            dtiExtra.IGBTtempLimitActive = dti6.IGBTtempLimitActive;
            dtiExtra.inputVoltageLimitActive = dti6.inputVoltageLimitActive;
            dtiExtra.motorAccelTempLimitActive = dti6.motorAccelTempLimitActive;
            dtiExtra.motorTempLimitActive = dti6.motorTempLimitActive;
            dtiExtra.RPMminLimitActive = dti6.RPMminLimitActive;
            dtiExtra.RPMmaxLimitActive = dti6.RPMmaxLimitActive;
            dtiExtra.powerLimitActive = dti6.powerLimitActive;
            dtiExtra.zero1 = dti6.zero1;
            dtiExtra.zero2 = dti6.zero2;
            dtiExtra.zero3 = dti6.zero3;
            dtiExtra.zero4 = dti6.zero4;
            dtiExtra.zero5 = dti6.zero5;
            taskEXIT_CRITICAL();
            break;
        }
        case PKT_7_ID: {
            PKT_DTI7 dti7 = {0};
            memcpy(&dti7, &rx_data, sizeof(dti7));
            taskENTER_CRITICAL();
            dtiData.maxAC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti7.maxAC_Current)) *
                DTI_16_SCALE;
            dtiData.avMaxAC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti7.avMaxAC_Current)) *
                DTI_16_SCALE;
            dtiData.minAC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti7.minAC_Current)) *
                DTI_16_SCALE;
            dtiData.avMinAC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti7.avMinAC_Current)) *
                DTI_16_SCALE;
            taskEXIT_CRITICAL();
            break;
        }
        case PKT_8_ID: {
            PKT_DTI8 dti8 = {0};
            memcpy(&dti8, &rx_data, sizeof(dti8));
            taskENTER_CRITICAL();
            dtiData.maxDC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti8.maxDC_Current)) *
                DTI_16_SCALE;
            dtiData.avMaxDC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti8.avMaxDC_Current)) *
                DTI_16_SCALE;
            dtiData.minDC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti8.minDC_Current)) *
                DTI_16_SCALE;
            dtiData.avMinDC_Current =
                (float)((int16_t)CHANGE_ENDIANESS_16(dti8.avMinDC_Current)) *
                DTI_16_SCALE;
            taskEXIT_CRITICAL();
            break;
        }
        case pcc_ID: {
            // Serial.print("PCC OK?");
            PCC_ProcessPCCMessage(&rx_data);
            break;
        }

        case mOBMS1_ID: {
            // > 8 bytes - FIX
            // OBMS1 raw = {0};
            // memcpy(&raw, &rx_data, sizeof(raw));

            // taskENTER_CRITICAL();
            // bmsData.packCurrent = raw.packCurrent * 0.1F;
            // bmsData.packVoltage = raw.packVoltage * 0.1F;
            // bmsData.soc = raw.packSOC * 0.5F;
            // bmsData.relayState = raw.relayState;
            // taskEXIT_CRITICAL();
            break;
        }

        case mOBMS2_ID: {
            OBMS2 raw = {0};
            memcpy(&raw, &rx_data, sizeof(raw));

            taskENTER_CRITICAL();
            bmsData.dischargeLimit = (float)raw.packDCL; // Drive Current Limits
            bmsData.chargeLimit = (float)raw.packCCL;    // Regen Current Limits
            bmsData.highTemp = raw.highTemp;
            bmsData.lowTemp = raw.lowTemp;
            taskEXIT_CRITICAL();
            break;
        }

        case mOBMS3_ID: {
            OBMS3 raw = {0};
            memcpy(&raw, &rx_data, sizeof(raw));

            taskENTER_CRITICAL();
            bmsData.lowCellVolt = raw.lowCellVolt * 0.0001F;
            bmsData.highCellVolt = raw.highCellVolt * 0.0001F;
            bmsData.avgCellVolt = raw.avgCellVolt * 0.0001F;
            bmsData.lowCellID = raw.lowCellVoltID;
            taskEXIT_CRITICAL();
            break;
        }
        case mIMD_GENERAL_ID: {
            IMD_General raw = {0};
            memcpy(&raw, &rx_data, sizeof(raw));

            taskENTER_CRITICAL();
            imdData.resistance = (float)raw.R_iso_corrected;
            imdData.status = raw.status_flags;
            // Bit 4 is "Iso alarm" per section 2.3 GET commands note 1)*
            imdData.isolation_fault =
                (raw.status_flags & (1 << 4)) ? true : false;
            taskEXIT_CRITICAL();
            break;
        }

        case mIMD_VOLTAGE_ID: {
            IMD_Voltage raw = {0};
            memcpy(&raw, &rx_data, sizeof(raw));
            taskENTER_CRITICAL();
            // formula: (RawValue - Offset) * Resolution
            // Note: If raw is 32128, voltage is 0V.
            imdData.hv_voltage = (raw.hv_system - 32128) * 0.05F;
            taskEXIT_CRITICAL();
            break;
        }
        default: {
            break;
        }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

dtiData1 *DTI_GetDTIData() { return &dtiData; }
dtiData2 *DTI_GetDTI_ExtraData() { return &dtiExtra; }

OrionBMSData *BMS_GetOrionData() { return &bmsData; }

IMDData *IMD_GetInfo() { return &imdData; }

// checksum = (byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6) XOR 0xFF
uint8_t ComputeChecksum(uint8_t *data) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 7; i++) {
        sum += data[i];
    }
    return sum ^ 0xFF;
}
