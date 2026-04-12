// equivalent to motor.cpp but only motor stuff. torque mapping will go here but
// vehicle state goes in vcu.cpp

#include "dti.h"
// #include "tc.h"
// #include "lc.h"

static DTIControlMode *internalMode = nullptr;

/**
 * Link internal control mode to defined controle mode in vehicle.cpp
 */
void DTI_LinkControlMode(DTIControlMode *mode) { internalMode = mode; }

/**
 * Sends the enable command periodically to allow for motor output
 * @param enable true if RTD_button is pressed
 *
 * */
void DTI_SendEnableCommand(bool enable) {
    DTIMessage enableMsg;
    enableMsg.id = ((PKT_SetBrakeCurrent_ID << 8) | DTI_NODE_ID);
    enableMsg.dlc = 1;
    if (enable) {
        enableMsg.data.bytes[0] = 1;
    } else {
        enableMsg.data.bytes[0] = 0;
    }
    CAN_Send(&enableMsg);
}

/**
 * Adjusts CAN_IDs only, not data values themselves. Sends over CANbus
 * @param value should be appropriately scaled for control mode
 *  */
void DTI_SendAccelCommand(float value) {
    if (internalMode == nullptr)
        return;

    DTIMessage throttleMsg;
    throttleMsg.is_bitfield = false;

    switch (*internalMode) {
    case AC_ONLY: {
        throttleMsg.id = ((PKT_SetRelativeCurrent_ID << 8) | DTI_NODE_ID);
        throttleMsg.dlc = 2;
        throttleMsg.data.half[0] = (int16_t)(value * 10);
    } break;
    case SPEED: {
        throttleMsg.id = ((PKT_SetERPM_ID << 8) | DTI_NODE_ID);
        throttleMsg.dlc = 4;
        throttleMsg.data.word[0] = (int16_t)(value * 10);
    } break;
    default: {
        throttleMsg.id = ((PKT_SetRelativeCurrent_ID << 8) | DTI_NODE_ID);
        throttleMsg.dlc = 2;
        throttleMsg.data.half[0] = (int16_t)(value * 10);
    } break;
    }

    CAN_Send(&throttleMsg);
}

/**
 * Adjusts CAN_IDs only, not data values themselves. Sends over CANbus
 * @param value should be appropriately scaled for control mode
 *  */
void DTI_SendBrakeCommand(float value) {
    DTIMessage brakeMsg;
    brakeMsg.id = ((PKT_SetBrakeCurrent_ID << 8) | DTI_NODE_ID);
    brakeMsg.dlc = 2;
    brakeMsg.is_bitfield = false;
    brakeMsg.data.half[0] = (int16_t)(value * 10);
    CAN_Send(&brakeMsg);
}
