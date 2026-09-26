// Anteater Electric Racing, 2025

#include "imd.h"
#include <arduino_freertos.h>
#include <string.h>

static IMDData imdData;

void IMD_HandleGeneralMessage(const uint8_t *data, uint8_t length) {
    if (length < sizeof(IMD_General))
        return;

    IMD_General raw;
    memcpy(&raw, data, sizeof(raw));

    taskENTER_CRITICAL();
    imdData.resistance = (float)raw.R_iso_corrected;
    imdData.status = raw.status_flags;
    // Bit 4 is "Iso alarm" per section 2.3 GET commands note 1)
    imdData.isolation_fault = (raw.status_flags & (1 << 4)) ? true : false;
    taskEXIT_CRITICAL();
}

void IMD_HandleVoltageMessage(const uint8_t *data, uint8_t length) {
    if (length < sizeof(IMD_Voltage))
        return;

    IMD_Voltage raw;
    memcpy(&raw, data, sizeof(raw));

    taskENTER_CRITICAL();
    // formula: (RawValue - Offset) * Resolution. Raw of 32128 = 0V.
    imdData.hv_voltage = (raw.hv_system - 32128) * 0.05F;
    taskEXIT_CRITICAL();
}

IMDData *IMD_GetInfo() { return &imdData; }