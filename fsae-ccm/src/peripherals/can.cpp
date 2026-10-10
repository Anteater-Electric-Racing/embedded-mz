// Anteater Electric Racing, 2025

#define THREAD_CAN_STACK_SIZE 128
#define THREAD_CAN_PRIORITY 1

#include <cstdint>

#include <FlexCAN_T4.h>
#include <arduino_freertos.h>

#include "peripherals/can.h"
#include "utils/utils.h"
#include "vehicle/comms/bus.h"
#include "vehicle/comms/telemetry.h"
#include "vehicle/devices/dti.h"
#include "vehicle/vcu.h"

#define CAN_INSTANCE CAN1 // can be removed
#define CAN_BAUD_RATE 500000

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

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
}

void CAN_Send(uint32_t id, uint64_t msg) {
    CAN_message_t tx_msg = {};
    tx_msg.id = id;
    tx_msg.len = 8;
    tx_msg.flags.extended = 0;
    memcpy(tx_msg.buf, &msg, sizeof(msg));

    // TODO: maybe keep here as broadcast? - watch CAN utilization
    can3.write(tx_msg);
    can2.write(tx_msg);
}

// overload speific to DTI_message
void CAN_Send(DTIMessage *msg) {
    if (msg == nullptr || msg->dlc > 8)
        return;
    CAN_message_t tx_msg = {};
    tx_msg.id = ((msg->id) << 8 | DTI_NODE_ID);
    tx_msg.len = msg->dlc;
    tx_msg.flags.extended = 1;

    // swap to bigEndian before sending to DTI
    if (msg->dlc <= 1 || msg->is_bitfield) {
        memcpy(tx_msg.buf, msg->data.bytes, msg->dlc);
    } else if (msg->dlc == 2) {
        uint16_t swapped = SWAP_16(msg->data.half[0]);
        memcpy(tx_msg.buf, &swapped, 2);
    } else if (msg->dlc == 4) {
        uint32_t swapped = SWAP_32(msg->data.word[0]);
        memcpy(tx_msg.buf, &swapped, 4);
    } else {
        // Fallback for 8-byte or odd lengths
        uint64_t swapped = SWAP_64(msg->data.raw);
        memcpy(tx_msg.buf, &swapped, 8);
    }

    // tx_msg.id = 0x0C41;
    // tx_msg.flags.extended = 1;
    // tx_msg.len = 1;
    // tx_msg.buf[0] = 0x01; // High byte of 1000 (0x03E8)

    // TODO: ksthakkar fix duplicate writes - watch CAN utilization
    can3.write(tx_msg);
    can2.write(tx_msg);
}

void CAN_Receive(uint32_t *rx_id, uint64_t *rx_data) {
    if (can3.read(rx_msg) || can2.read(rx_msg)) {
        *rx_id = rx_msg.id;
        memcpy(rx_data, rx_msg.buf, sizeof(*rx_data));
        canLatestHealthyStateTime =
            xTaskGetTickCount(); // update time for successful CAN
    } else {                     // No message received, assign default values
        *rx_id = 0;
        *rx_data = 0;
    }
}

bool CAN_SendFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length) {
    if ((bus != 2 && bus != 3) || id > 0x7FF || length > 8 || data == nullptr)
        return false;
    CAN_message_t frame = {};
    frame.id = id;
    frame.len = length;
    frame.flags.extended = 0;
    memcpy(frame.buf, data, length);
    return (bus == 2 ? can2.write(frame) : can3.write(frame)) != 0;
}
