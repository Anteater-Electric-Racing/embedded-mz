// equivalent to motor.cpp but only motor stuff. torque mapping will go here but
// vehicle state goes in vcu.cpp

#include "dti.h"
#include "peripherals/can.h"

// #include "tc.h"
// #include "lc.h"

static DTIControlMode *internalMode = nullptr;

/**
 * Link internal control mode to defined controle mode in vehicle.cpp
 */
void DTI_LinkControlMode(DTIControlMode *mode) { internalMode = mode; }

void DTI_SetACLimits(float max, float min) {
    DTIMessage acMax;
    DTIMessage acMin;

    acMax.id = ((PKT_SetMaxCurrentAC_ID));
    acMax.dlc = 2;
    acMax.is_bitfield = false;
    acMax.data.half[0] = (int16_t)(max * 10);

    acMin.id = ((PKT_SetMaxBrakeCurrentAC_ID));
    acMin.dlc = 2;
    acMin.is_bitfield = false;
    acMin.data.half[0] = (int16_t)(min * 10);

    DTICAN_Send(&acMax);
    DTICAN_Send(&acMin);
}
void DTI_SetDCLimits(float max, float min) {
    DTIMessage dcMax;
    DTIMessage dcMin;

    dcMax.id = ((PKT_SetMaxCurrentDC_ID));
    dcMax.dlc = 2;
    dcMax.is_bitfield = false;
    dcMax.data.half[0] = (int16_t)(max * 10);

    dcMin.id = ((PKT_SetMaxBrakeCurrentDC_ID));
    dcMin.dlc = 2;
    dcMin.is_bitfield = false;
    dcMin.data.half[0] = (int16_t)(min * 10);

    DTICAN_Send(&dcMax);
    DTICAN_Send(&dcMin);
}
/**
 * Sends the enable command periodically to allow for motor output
 * @param enable true if RTD_button is pressed
 * */
void DTI_SendEnableCommand(bool enable) {
    DTIMessage enableMsg;
    enableMsg.id = ((PKT_SetDriveEnable_ID));
    enableMsg.dlc = 1;
    if (enable) {
        enableMsg.data.bytes[0] = 1;
    } else {
        enableMsg.data.bytes[0] = 0;
    }
    DTICAN_Send(&enableMsg);
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
    case TORQUE: {
        throttleMsg.id = ((PKT_SetRelativeCurrent_ID));
        throttleMsg.dlc = 2;
        throttleMsg.data.half[0] = (int16_t)(value * 10);
    } break;
    case SPEED: {
        throttleMsg.id = ((PKT_SetERPM_ID));
        throttleMsg.dlc = 4;
        throttleMsg.data.word[0] = (int16_t)(value * 10);
    } break;
    default: {
        throttleMsg.id = ((PKT_SetRelativeCurrent_ID));
        throttleMsg.dlc = 2;
        throttleMsg.data.half[0] = (int16_t)(value * 10);
    } break;
    }

    DTICAN_Send(&throttleMsg);
}

/**
 * Adjusts CAN_IDs only, not data values themselves. Sends over CANbus
 * @param value should be appropriately scaled for control mode
 *  */
void DTI_SendBrakeCommand(float value) {
    DTIMessage brakeMsg;
    brakeMsg.id = ((PKT_SetBrakeCurrent_ID));
    brakeMsg.dlc = 2;
    brakeMsg.is_bitfield = false;
    brakeMsg.data.half[0] = (int16_t)(value * 10);
    DTICAN_Send(&brakeMsg);
}
