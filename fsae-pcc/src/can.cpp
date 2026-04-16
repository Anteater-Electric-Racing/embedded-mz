// Anteater Electric Racing, 2025

#define THREAD_CAN_STACK_SIZE 128
#define THREAD_CAN_PRIORITY 1

#include "can.h"
#include <FlexCAN_T4.h>
#include <arduino_freertos.h>

#define CAN_BAUDRATE 500000
#define PCC_CAN_ID 0x222
// #define PCC_CAN_ID 0x123

// charger can ids
#define BMS_CHARGER_STATUS_CAN_ID 0x185
#define BMS_CHARGER_CMD_CAN_ID 0x306
// charger safety byte
#define CHARGER_SAFETY_BYTE 6
// charger safety mask
#define CHARGER_SAFETY_MASK 0x08
// bms can timeout
#define BMS_CAN_TIMEOUT_MS 1500U

typedef struct __attribute__((packed)) {
    uint8_t state;               // Precharge state
    uint8_t errorCode;           // Error code
    uint16_t accumulatorVoltage; // Accumulator voltage in volts
    uint16_t tsVoltage;          // Transmission side voltage in volts
    uint16_t prechargeProgress;  // Precharge progress in percent
} PCC;

static PCC pccData;

// struct for charger data
typedef struct {
    bool safetyActive;
    float packVoltage;
    float packCCL;
    uint8_t rollingCounter;
    uint32_t lastRxTime;
} ChargerData;

// struct for charger data declaration
static ChargerData chargerData;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
static CAN_message_t pccMsg;

void CAN_Init() {
    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE); // Set CAN baud rate
    can2.setTX(DEF);
    can2.setRX(DEF);
    can2.enableFIFO();
    // set fifo filters for charger can ids
    can2.setFIFOFilter(REJECT_ALL);
    can2.setFIFOFilter(0, BMS_CHARGER_STATUS_CAN_ID, STD);
    can2.setFIFOFilter(1, BMS_CHARGER_CMD_CAN_ID, STD);

    chargerData = {0};
}

void CAN_SendPCCMessage(uint8_t state, uint8_t errorCode,
                        float accumulatorVoltage, float tsVoltage,
                        float prechargeProgress) {
    pccData = {0};

    pccData = {.state = state,
               .errorCode = errorCode,
               .accumulatorVoltage = uint16_t(accumulatorVoltage * 100),
               .tsVoltage = uint16_t(tsVoltage * 100),
               .prechargeProgress = uint16_t(prechargeProgress)};

    // pccData.accumulatorVoltage = 1;

    pccMsg.id = PCC_CAN_ID;

    memcpy(pccMsg.buf, &pccData, sizeof(PCC));
    can2.write(pccMsg);

    // pccData.accumulatorVoltage = 1.0;
    // Serial.println("CAN message sent");

    // Serial.print("Accumulator Voltage | ");
    // Serial.print(pccData.accumulatorVoltage);

    // Serial.print("Precharge Progress | ");
    // Serial.print(pccData.prechargeProgress);
    // Serial.print('\n');
}

void CAN_PollMessages() {
    CAN_message_t rxMsg;

    while (can2.read(rxMsg)) {
        // read charger status can id
        if (rxMsg.id == BMS_CHARGER_STATUS_CAN_ID) {
            // set charger safety active
            chargerData.safetyActive =
                (rxMsg.buf[CHARGER_SAFETY_BYTE] & CHARGER_SAFETY_MASK) != 0;
            chargerData.lastRxTime = millis();    
        } else if (rxMsg.id == BMS_CHARGER_CMD_CAN_ID) {
            // check if charger safety is active
            if (!chargerData.safetyActive) {
                continue;
            }
            uint16_t rawVoltage =
                ((uint16_t)rxMsg.buf[0] << 8) | rxMsg.buf[1];
            chargerData.packVoltage = rawVoltage / 10.0F;
            chargerData.packCCL = rxMsg.buf[2] * 8.0F / 10.0F;
            chargerData.rollingCounter = rxMsg.buf[6];
        }
    }
}

// check if charger safety is active
bool CAN_IsChargerSafetyActive() {
    bool active;
    taskENTER_CRITICAL();
    active = chargerData.safetyActive;
    taskEXIT_CRITICAL();
    return active;
}

// get last bms can rx time
uint32_t CAN_GetBMSLastRxTime() {
    uint32_t t;
    taskENTER_CRITICAL();
    t = chargerData.lastRxTime;
    taskEXIT_CRITICAL();
    return t;
}

// get charger voltage
float CAN_GetChargerVoltage() {
    float v;
    taskENTER_CRITICAL();
    v = chargerData.packVoltage;
    taskEXIT_CRITICAL();
    return v;
}

// get charger ccl
float CAN_GetChargerCCL() {
    float ccl;
    taskENTER_CRITICAL();
    ccl = chargerData.packCCL;
    taskEXIT_CRITICAL();
    return ccl;
}

// get charger counter
uint8_t CAN_GetChargerCounter() {
    uint8_t c;
    taskENTER_CRITICAL();
    c = chargerData.rollingCounter;
    taskEXIT_CRITICAL();
    return c;
}
