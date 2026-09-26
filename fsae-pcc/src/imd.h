#pragma once

#include <stdint.h>

// IMD (Bender/Isobender) CAN IDs -- extended 29-bit frames, same as bus.h
#define mIMD_GENERAL_ID 0x18FF01F4
#define mIMD_VOLTAGE_ID 0x18FF03F4

typedef struct __attribute__((packed)) {
    uint16_t R_iso_corrected; // [kOhm] Intel order
    uint8_t R_iso_status;     // 0xFC: Startup, 0xFD: First Meas, 0xFE: Normal
    uint8_t measurement_cnt;
    uint16_t status_flags;   // Warnings/Alarms (Bit 0: Error, Bit 4: Iso Alarm)
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

typedef struct {
    float resistance; // kOhm
    float hv_voltage; // Volts
    uint16_t status;  // Raw flags
    bool isolation_fault;
} IMDData;

// Called by can.cpp when the matching message ID arrives.
void IMD_HandleGeneralMessage(const uint8_t *data, uint8_t length);
void IMD_HandleVoltageMessage(const uint8_t *data, uint8_t length);

IMDData *IMD_GetInfo();