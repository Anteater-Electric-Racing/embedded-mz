// TO REMOVE

#define THREAD_CAN_STACK_SIZE 128
#define THREAD_CAN_PRIORITY 1

#include "pcc.h"
#include "arduino_freertos.h"

static PCC pccData;

void PCC_Init() {
    pccData = {.state = 0,
               .errorCode = 0,
               .accumulatorVoltage = 0,
               .tsVoltage = 0,
               .prechargeProgress = 404};
}

// TODO: Put in bus instead of seperate file
void PCC_ProcessPCCMessage(uint64_t *rx_data) {
    taskENTER_CRITICAL();
    memcpy(&pccData, rx_data, sizeof(pccData));
    taskEXIT_CRITICAL();
}

PCC *PCC_GetData() { return &pccData; }

// TODO Define constants for progress Threshold and state defs
bool PCC_PrechargeComplete() {
    return (PCC_GetData()->prechargeProgress >= 90 &&
            PCC_GetData()->state == 3);
}
