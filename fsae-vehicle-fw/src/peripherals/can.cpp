// Anteater Electric Racing, 2025

#define THREAD_CAN_STACK_SIZE 128
#define THREAD_CAN_PRIORITY 1

#include <cstdint>
#include <isotp.h>

#include <FlexCAN_T4.h>
#include <arduino_freertos.h>

#include "peripherals/can.h"
#include "utils/utils.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/devices/dti.h"
#include "vehicle/vcu.h"

#define CAN_INSTANCE CAN1 // can be removed
#define CAN_BAUD_RATE 500000

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
isotp<RX_BANKS_16, 512> tp;

CAN_message_t tx_msg;
CAN_message_t rx_msg;

#define CAN_TIMEOUT_MS 100

void CAN_Init() {
    // Initialize CAN bus
    can2.begin();
    can2.setBaudRate(CAN_BAUD_RATE);
    can2.setTX(DEF);
    can2.setRX(DEF);
    can2.enableFIFO();

    can3.begin();
    can3.setBaudRate(CAN_BAUD_RATE);
    can3.setTX(DEF);
    can3.setRX(DEF);
    can3.enableFIFO();

    tp.begin();
    tp.setWriteBus(&can3); // Set the bus to write to can3
}

void CAN_Send(uint32_t id, uint64_t msg) {
    tx_msg.id = id;
    memcpy(tx_msg.buf, &msg, sizeof(msg));

    // @ksthakkar TODO: fix duplicate writes - watch CAN utilization
    can3.write(tx_msg);
    can2.write(tx_msg);
}

void CAN_Receive(uint32_t *rx_id, uint64_t *rx_data) {
    if (can3.read(rx_msg) || can2.read(rx_msg)) {
        *rx_id = rx_msg.id;
        memcpy(rx_data, rx_msg.buf, sizeof(*rx_data));
    } else { // No message received, assign default values
        *rx_id = 0;
        *rx_data = 0;
    }
}

void CAN_ISOTP_Send(uint32_t id, uint8_t *msg, uint16_t size) {
    ISOTP_data config;
    config.id = id;
    config.flags.extended = 0; // Standard frame
    config.separation_time =
        1; // Time between back-to-back frames in milliseconds
    tp.write(config, msg, size);
}
