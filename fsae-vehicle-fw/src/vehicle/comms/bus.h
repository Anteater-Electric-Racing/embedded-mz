#pragma once

#include <stdint.h>

// similar to ifl.h where can structs, msgs ids, and pointer functions are
// defined.

// CAN IDs
/*
 * stick to standard ID
 * define necessary packed IDs...
 * node ID not needed
 *
 * DTI IS BIG ENDIAN.
 * ALL messages recieved must be converted to little endian before processing
 * ALL outgoing messages must be converted to big endian before sending
 */

// General ID TODO
#define mDTI_ID 0x65
// PKT_R = Packet Recieve

typedef enum {
    PKT_1_ID = 0x1F,
    PKT_2_ID = 0x20,
    PKT_3_ID = 0x21,
    PKT_4_ID = 0x22,
    PKT_5_ID = 0x23,
    PKT_6_ID = 0x24,
    PKT_7_ID = 0x25,
    PKT_8_ID = 0x26
} DTI_Recieve_ID;

#define pcc_ID 0x222

#define mOBMS1_ID 0x6B0
#define mOBMS2_ID 0x6B1
#define mOBMS3_ID 0x6B2
#define mOBMS4_ID 0x6B3

// double check this
#define mIMD_GENERAL_ID 0x18FF01F4
#define mIMD_DETAIL_ID 0x18FF02F4
#define mIMD_VOLTAGE_ID 0x18FF03F4
#define mIMD_IT_SYSTEM_ID 0x18FF04F4

// J1939 ID: Priority 6 (0x18) | PGN (0xEF00) | Dest (0xF4) | Source (0x17)
#define IMD_CMD_ID 0x18EFF417

// Indices from manual section 2.2
#define IMD_IDX_THRESHOLD_ERROR 0x47
#define IMD_IDX_STATUS_LOCK 0x6B

// Lock States
#define IMD_LOCK_UNLOCK 0xFC
#define IMD_LOCK_LOCK 0xFD

// ====== DTI Inverter Packets ======= //
typedef struct __attribute__((packed)) {
    uint8_t controlMode;
    // 1: CONTROL_MODE_SPEED
    // 2: CONTROL_MODE_CURRENT
    // 3: CONTROL_MODE_CURRENT_BRAKE
    // 4: CONTROL_MODE_POS
    // 7: CONTROL_MODE_NONE
    // 0, 5, 6: NOT USED
    int16_t targetLq;
    int16_t motorPosition; // in degrees
    int8_t isMotorStill;   // in still position or not
    int16_t reserved;
} PKT_DTI1;

typedef struct __attribute__((packed)) {
    int32_t eRPM;         // eRPM = motor RPM * number of motor pole pairs
    int16_t dutyCycle;    // controller duty cycle
    int16_t inputVoltage; // dcVoltage
} PKT_DTI2;

typedef struct __attribute__((packed)) {
    int16_t acCurrent; // AC motor current (sign is regen or running)
    int16_t dcCurrent; // AC motor current (sign is regen or running)
    int32_t reserved;
} PKT_DTI3;

typedef struct __attribute__((packed)) {
    int16_t controllerTemp; // temp of inverter semiconductors
    int16_t motorTemp;      // temp of motor measured by inverter
    int8_t faultCode;       // all inverter faults, add to faultMAP TODO
    int8_t reserved;        //
    int16_t reserved2;
} PKT_DTI4;

typedef struct __attribute__((packed)) {
    int32_t focLd; // foc alg Ld
    int32_t focLq; // foc alg Lq
} PKT_DTI5;

typedef struct __attribute__((packed)) {
    int8_t throttleInput; // throttle straight to inverter
    int8_t brakeInput;    // throttle straight to inverter
    // byte 2
    bool digitalIn1 : 1; // digital input
    bool digitalIn2 : 1; // digital input
    bool digitalIn3 : 1;
    bool digitalIn4 : 1;  // digital input
    bool digitalOut1 : 1; // digital output
    bool digitalOut2 : 1; // digital output
    bool digitalOut3 : 1; // digital output
    bool digitalOut4 : 1; // digital output
    uint8_t driveEnabled; // RTM toggle
    // byte 4
    bool capTempLimitActive : 1;
    bool dcTempLimitActive : 1;

    bool driveEnableLimitActive : 1;
    bool IGBTaccelLimitActive : 1;
    bool IGBTtempLimitActive : 1;
    bool inputVoltageLimitActive : 1;
    bool motorAccelTempLimitActive : 1;
    bool motorTempLimitActive : 1;
    // byte 5
    bool RPMminLimitActive : 1;
    bool RPMmaxLimitActive : 1;
    bool powerLimitActive : 1;

    // zeros
    bool zero1 : 1;
    bool zero2 : 1;
    bool zero3 : 1;
    bool zero4 : 1;
    bool zero5 : 1;
    uint8_t reserved;
    uint8_t CANmapVersion;

} PKT_DTI6;

typedef struct __attribute__((packed)) {
    int16_t maxAC_Current;
    int16_t avMaxAC_Current;
    int16_t minAC_Current;
    int16_t avMinAC_Current;
} PKT_DTI7;

typedef struct __attribute__((packed)) {
    int16_t maxDC_Current;
    int16_t avMaxDC_Current;
    int16_t minDC_Current;
    int16_t avMinDC_Current;
} PKT_DTI8;

typedef struct {
    // combined dti data essential readings
    uint8_t controlMode;
    // 1: CONTROL_MODE_SPEED
    // 2: CONTROL_MODE_CURRENT
    // 3: CONTROL_MODE_CURRENT_BRAKE
    // 4: CONTROL_MODE_POS
    // 7: CONTROL_MODE_NONE
    // 0, 5, 6: NOT USED
    float targetLq;
    float motorPosition;  // in degrees
    uint8_t isMotorStill; // in still position or not
    float eRPM;           // eRPM = motor RPM * number of motor pole pairs
    float dutyCycle;      // controller duty cycle
    float inputVoltage;   // dcVoltage
    float acCurrent;      // AC motor current (sign is regen or running)
    float dcCurrent;      // AC motor current (sign is regen or running)
    float controllerTemp; // temp of inverter semiconductors
    float motorTemp;      // temp of motor measured by inverter
    uint8_t faultCode;    // all inverter faults, add to faultMAP TODO
    float focLd;          // foc alg Ld
    float focLq;          // foc alg lq

    uint8_t driveEnabled; // RTM toggle

    float maxAC_Current;
    float avMaxAC_Current;
    float minAC_Current;
    float avMinAC_Current;
    float maxDC_Current;
    float avMaxDC_Current;
    float minDC_Current;
    float avMinDC_Current;

} dtiData1;

typedef struct {
    // combined dti data nonessential readings

    float throttleInput; // throttle straight to inverter
    float brakeInput;
    // byte 2
    bool digitalIn1;  // digital input
    bool digitalIn2;  // digital input
    bool digitalIn3;  // digital input
    bool digitalIn4;  // digital input
    bool digitalOut1; // digital output
    bool digitalOut2; // digital output
    bool digitalOut3; // digital output
    bool digitalOut4; // digital output
    // byte 4
    bool capTempLimitActive;
    bool dcTempLimitActive;
    bool driveEnableLimitActive;
    bool IGBTaccelLimitActive;
    bool IGBTtempLimitActive;
    bool inputVoltageLimitActive;
    bool motorAccelTempLimitActive;
    bool motorTempLimitActive;
    // byte 5
    bool RPMminLimitActive;
    bool RPMmaxLimitActive;
    bool powerLimitActive;

    // zeros
    bool zero1;
    bool zero2;
    bool zero3;
    bool zero4;
    bool zero5;

    uint8_t reserved;
    uint8_t CANmapVersion;
} dtiData2;

// ============ Orion BMS, Isobender IMD ========================= //
typedef struct __attribute__((packed)) {
    int32_t packCurrent;  // Byte 0-1: Pack Current (0.1A/bit)
    uint16_t packVoltage; // Byte 2-3: Pack Voltage (0.1V/bit)
    uint8_t packSOC;      // Byte 4: SOC (0.5%/bit)
    uint8_t relayState;   // Byte 5: Relay State Bitmask
    uint8_t reserved;     // Byte 6
    uint8_t checksum;     // Byte 7
} OBMS1;

typedef struct __attribute__((packed)) {
    uint16_t packDCL; // Byte 0-1: Discharge Current Limit (1A/bit)
    uint16_t packCCL; // Byte 2-3: Charge Current Limit (1A/bit)
    int8_t highTemp;  // Byte 4: Highest Temp (1C/bit)
    int8_t lowTemp;   // Byte 5: Lowest Temp (1C/bit)
    uint8_t reserved; // Byte 6
    uint8_t checksum; // Byte 7
} OBMS2;

typedef struct __attribute__((packed)) {
    uint16_t lowCellVolt;  // Byte 0-1: Lowest Cell Voltage (0.0001V/bit)
    uint16_t highCellVolt; // Byte 2-3: Highest Cell Voltage (0.0001V/bit)
    uint16_t avgCellVolt;  // Byte 4-5: Average Cell Voltage (0.0001V/bit)
    uint8_t lowCellVoltID; // Byte 6: ID of the lowest voltage cell
    uint8_t checksum;      // Byte 7
} OBMS3;

typedef struct __attribute__((packed)) {
    uint16_t packResistance;  // Example: Pack Internal Resistance
    uint16_t packOpenVoltage; // Example: Pack Open Circuit Voltage
    uint8_t reserved[3];
    uint8_t checksum;
} OBMS4;

typedef struct {
    float packCurrent;  // Amps
    float packVoltage;  // Volts
    float soc;          // Percentage (0-100%)
    uint8_t relayState; // Bitmask (Discharge, Charge, etc.)

    float dischargeLimit; // Amps
    float chargeLimit;    // Amps
    int8_t highTemp;      // Celsius
    int8_t lowTemp;       // Celsius

    float lowCellVolt;  // Volts (e.g., 3.4215f)
    float highCellVolt; // Volts
    float avgCellVolt;  // Volts
    uint8_t lowCellID;
} OrionBMSData;

typedef struct __attribute__((packed)) {
    uint16_t R_iso_corrected; // [kOhm] Intel order
    uint8_t R_iso_status;     // 0xFC: Startup, 0xFD: First Meas, 0xFE:
    // Normal
    uint8_t measurement_cnt;
    uint16_t status_flags; // Warnings and
    // Alarms (Bit 0: Error, Bit 4: Iso
    //                          // Alarm, etc.)
    uint8_t device_activity; // 0: Init, 1: Normal, 2: Self-test
    uint8_t reserved;
} IMD_General;

typedef struct __attribute__((packed)) {
    uint16_t hv_system;       // Offset 32128, 0.05V/bit
    uint16_t hv_neg_to_earth; // Offset 32128, 0.05V/bit
    uint16_t hv_pos_to_earth; // Offset 32128, 0.05V/bit
    uint8_t measurement_cnt;
    uint8_t reserved;
} IMD_Voltage;

// Unified storage for your application
typedef struct {
    float resistance; // kOhm
    float hv_voltage; // Volts
    uint16_t status;  // Raw flags
    bool isolation_fault;
} IMDData;

uint8_t ComputeChecksum(uint8_t *data);

OrionBMSData *BMS_GetOrionData();
IMDData *IMD_GetInfo();
dtiData1 *DTI_GetDTIData();
dtiData2 *DTI_GetDTI_ExtraData();

void Bus_Init();
void threadBus(void *pvParameters);
