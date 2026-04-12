/**
 * Anteater Electric Racing, 2026
 * Description:
 *
 **/

#pragma once

#include "peripherals/can.h"
#include "utils/utils.h"
#include <stdint.h>

#define DTI_NODE_ID 0x65
typedef enum {
    PKT_SetCurrent_ID = 0x01,
    PKT_SetBrakeCurrent_ID = 0x02,
    PKT_SetERPM_ID = 0x03,
    PKT_SetPosition_ID = 0x04,
    PKT_SetRelativeCurrent_ID = 0x05,
    PKT_SetRelativeBrakeCurrent_ID = 0x06,
    PKT_SetDigIO_ID = 0x07,
    PKT_SetMaxCurrentAC_ID = 0x08,
    PKT_SetMaxBrakeCurrentAC_ID = 0x09,
    PKT_SetMaxCurrentDC_ID = 0x0A,
    PKT_SetMaxBrakeCurrentDC_ID = 0x0B,
    PKT_SetDriveEnable_ID = 0x0C
} DTISendID;

typedef enum { AC_ONLY, SPEED } DTIControlMode;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    bool is_bitfield;
    union {
        uint64_t raw;     // For 8-byte transfers
        uint32_t word[2]; // For 4-byte chunks
        uint16_t half[4]; // For 2-byte chunks
        uint8_t bytes[8]; // Raw access
    } data;
} DTIMessage;

// TODO include a LUT for throttle values.

void DTILinkControlMode(DTIControlMode *mode);
void DTI_SendEnableCommand(bool enable);
void DTI_SendAccelCommand(float value);
void DTI_SendBrakeCommand(float value);
