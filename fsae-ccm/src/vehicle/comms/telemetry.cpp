// Anteater Electric Racing, 2025
#define TELEMETRY_CAN_ID 0x666 // Example CAN ID for telemetry messages
#define TELEMETRY_PERIOD_MS 10 // Telemetry update period in milliseconds

#include "vehicle/comms/telemetry.h"

#include <arduino_freertos.h>

TelemetryData telemetryData;

void Telemetry_Init() {
    telemetryData = {// Fill with reasonable dummy values
                     // Analog Data
                     .RTMState = 0,
                     .APPS_Travel = 0.0F,
                     .BSEFront = 0.0F,
                     .BSERear = 0.0F,
                     .BSEAvg = 0.0F,
                     .BRLinpot = 0.0F,
                     .FRLinpot = 0.0F,
                     .BLLinpot = 0.0F,
                     .FLLinpot = 0.0F,

                     .imdResistance = 0.0F,
                     .imdStatus = 0,

                     .packVoltage = 0.0F,
                     .packCurrent = 0.0F,
                     .soc = 0.0F,
                     .dischargeLimit = 0.0F,
                     .chargeLimit = 0.0F,
                     .lowCellVolt = 0.0F,
                     .highCellVolt = 0.0F,
                     .avgCellVolt = 0.0F,

                     .controlMode = 0,
                     .targetIq = 0.0F,
                     .motorPosition = 0.0F,
                     .isMotorStill = 0,
                     .eRPM = 0.0F,
                     .dutyCycle = 0.0F,
                     .inputVoltage = 0.0F,
                     .acCurrent = 0.0F,
                     .dcCurrent = 0.0F,
                     .controllerTemp = 0.0F,
                     .motorTemp = 0.0F,
                     .faultCode = 0,
                     .focId = 0.0F,
                     .focIq = 0.0F,

                     .driveEnabled = 0,

                     .maxAC_Current = 0.0F,
                     .avMaxAC_Current = 0.0F,
                     .minAC_Current = 0.0F,
                     .avMinAC_Current = 0.0F,
                     .maxDC_Current = 0.0F,
                     .avMaxDC_Current = 0.0F,
                     .minDC_Current = 0.0F,
                     .avMinDC_Current = 0.0F,

                     .dti_throttleInput = 0.0F,
                     .dti_brakeInput = 0.0F,

                     .digitalIn1 = 0,
                     .digitalIn2 = 0,
                     .digitalIn3 = 0,
                     .digitalIn4 = 0,
                     .digitalOut1 = 0,
                     .digitalOut2 = 0,
                     .digitalOut3 = 0,
                     .digitalOut4 = 0,

                     .capTempLimitActive = 0,
                     .dcTempLimitActive = 0,
                     .driveEnableLimitActive = 0,
                     .IGBTaccelLimitActive = 0,
                     .IGBTtempLimitActive = 0,
                     .inputVoltageLimitActive = 0,
                     .motorAccelTempLimitActive = 0,
                     .motorTempLimitActive = 0,

                     .RPMminLimitActive = 0,
                     .RPMmaxLimitActive = 0,
                     .powerLimitActive = 0,

                     .CANmapVersion = 0,
                     .vehicleState = STATE_OFF,

                     .faultMap = 0};
}

void threadTelemetry(void *pvParameters) {
    static TickType_t lastWakeTime =
        xTaskGetTickCount(); // Initialize the last wake time
    while (true) {
        taskENTER_CRITICAL(); // Enter critical section
        telemetryData = {
            // Fill with reasonable dummy values
            // Analog Data

            .RTMState = RTM_ButtonState(),
            .APPS_Travel = APPS_GetAPPSReading(),
            .BSEFront = BSE_GetBSEReading()->bseFront_Reading,
            .BSERear = BSE_GetBSEReading()->bseRear_Reading,
            .BSEAvg = BSE_GetBSEAverage(),
            .BRLinpots = Linpot_GetData()->shockTravel1_mm,
            .FRLinpots = Linpot_GetData()->shockTravel2_mm,
            .BLLinpots = Linpot_GetData()->shockTravel3_mm,
            .FLLinpots = Linpot_GetData()->shockTravel4_mm,

            .imdResistance = IMD_GetInfo()->resistance,
            .imdStatus = IMD_GetInfo()->status,

            .packVoltage = BMS_GetOrionData()->packVoltage,
            .packCurrent = BMS_GetOrionData()->packCurrent,
            .soc = BMS_GetOrionData()->soc,
            .dischargeLimit = BMS_GetOrionData()->dischargeLimit,
            .chargeLimit = BMS_GetOrionData()->chargeLimit,
            .lowCellVolt = BMS_GetOrionData()->lowCellVolt,
            .highCellVolt = BMS_GetOrionData()->highCellVolt,
            .avgCellVolt = BMS_GetOrionData()->avgCellVolt,

            .controlMode = DTI_GetDTIData()->controlMode,
            .targetIq = DTI_GetDTIData()->targetIq,
            .motorPosition = DTI_GetDTIData()->motorPosition,
            .isMotorStill = DTI_GetDTIData()->isMotorStill,
            .eRPM = DTI_GetDTIData()->eRPM,
            .dutyCycle = DTI_GetDTIData()->dutyCycle,
            .inputVoltage = DTI_GetDTIData()->inputVoltage,
            .acCurrent = DTI_GetDTIData()->acCurrent,
            .dcCurrent = DTI_GetDTIData()->dcCurrent,
            .controllerTemp = DTI_GetDTIData()->controllerTemp,
            .motorTemp = DTI_GetDTIData()->motorTemp,
            .faultCode = DTI_GetDTIData()->faultCode,
            .focId = DTI_GetDTIData()->focId,
            .focIq = DTI_GetDTIData()->focIq,

            .driveEnabled = DTI_GetDTIData()->driveEnabled,

            .maxAC_Current = DTI_GetDTIData()->maxAC_Current,
            .avMaxAC_Current = DTI_GetDTIData()->avMaxAC_Current,
            .minAC_Current = DTI_GetDTIData()->minAC_Current,
            .avMinAC_Current = DTI_GetDTIData()->avMinAC_Current,
            .maxDC_Current = DTI_GetDTIData()->maxDC_Current,
            .avMaxDC_Current = DTI_GetDTIData()->avMaxDC_Current,
            .minDC_Current = DTI_GetDTIData()->minDC_Current,
            .avMinDC_Current = DTI_GetDTIData()->avMinDC_Current,

            .dti_throttleInput = DTI_GetDTI_ExtraData()->throttleInput,
            .dti_brakeInput = DTI_GetDTI_ExtraData()->brakeInput,

            .digitalIn1 = DTI_GetDTI_ExtraData()->digitalIn1,
            .digitalIn2 = DTI_GetDTI_ExtraData()->digitalIn2,
            .digitalIn3 = DTI_GetDTI_ExtraData()->digitalIn3,
            .digitalIn4 = DTI_GetDTI_ExtraData()->digitalIn4,
            .digitalOut1 = DTI_GetDTI_ExtraData()->digitalOut1,
            .digitalOut2 = DTI_GetDTI_ExtraData()->digitalOut2,
            .digitalOut3 = DTI_GetDTI_ExtraData()->digitalOut3,
            .digitalOut4 = DTI_GetDTI_ExtraData()->digitalOut4,

            .capTempLimitActive = DTI_GetDTI_ExtraData()->capTempLimitActive,
            .dcTempLimitActive = DTI_GetDTI_ExtraData()->dcTempLimitActive,
            .driveEnableLimitActive =
                DTI_GetDTI_ExtraData()->driveEnableLimitActive,
            .IGBTaccelLimitActive =
                DTI_GetDTI_ExtraData()->IGBTaccelLimitActive,
            .IGBTtempLimitActive = DTI_GetDTI_ExtraData()->IGBTtempLimitActive,
            .inputVoltageLimitActive =
                DTI_GetDTI_ExtraData()->inputVoltageLimitActive,
            .motorAccelTempLimitActive =
                DTI_GetDTI_ExtraData()->motorAccelTempLimitActive,
            .motorTempLimitActive =
                DTI_GetDTI_ExtraData()->motorTempLimitActive,

            .RPMminLimitActive = DTI_GetDTI_ExtraData()->RPMminLimitActive,
            .RPMmaxLimitActive = DTI_GetDTI_ExtraData()->RPMmaxLimitActive,
            .powerLimitActive = DTI_GetDTI_ExtraData()->powerLimitActive,

            .CANmapVersion = DTI_GetDTI_ExtraData()->CANmapVersion,
            .vehicleState = STATE_OFF,

            .faultMap = Faults_GetFaults()};
        taskEXIT_CRITICAL();

        // Serial.print(telemetryData.faultMap);
        // Serial.print(" | ");
        // Serial.print(sizeof(TelemetryData));
        // Serial.print("\r");

        CAN_Send(0x520, (uint64_t)telemetryData.RTMState);

        uint8_t *serializedData = (uint8_t *)&telemetryData;
        CAN_ISOTP_Send(TELEMETRY_CAN_ID, serializedData, sizeof(TelemetryData));

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(
                TELEMETRY_PERIOD_MS)); // Delay until the next telemetry update
    }
}

TelemetryData const *Telemetry_GetData() { return &telemetryData; }
