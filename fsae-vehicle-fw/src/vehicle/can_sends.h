#pragma once

#include <stdint.h>

#define dtiNodeID 0x65 // TODO

typedef enum {
    PKT_1_ID = 0x1F,
    PKT_2_ID = 0x20,
    PKT_3_ID = 0x21,
    PKT_4_ID = 0x22,
    PKT_5_ID = 0x23,
    PKT_6_ID = 0x24,
    PKT_7_ID = 0x25,
    PKT_8_ID = 0x26
} DTI_Send;

// sending PKT ID
#define setCurrent_ID 0x01
#define setBrakeCurrent_ID 0x02
#define setERPM_ID 0x03
#define setPosition_ID 0x04
#define setRelativeCurrent_ID 0x05
#define setRelativeBrakeCurrent_ID 0x06
#define setDigIO_ID 0x07
#define setMaxCurrentAC_ID 0x08
#define setMaxBrakeCurrentAC_ID 0x09
#define setMaxCurrentDC_ID 0x0A
#define setMaxBrakeCurrentDC_ID 0x0B
#define setDriveEnable_ID 0x0C
