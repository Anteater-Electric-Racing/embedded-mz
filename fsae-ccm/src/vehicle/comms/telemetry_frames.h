// Named Classic CAN messages. Explicit encoders define the wire layout.
#pragma once
#include <stdint.h>
#include <string.h>
static_assert(sizeof(float) == 4, "Telemetry requires binary32 floats");
struct TelemetryFrame { uint32_t id; uint8_t data[8]; };
namespace TelemetryMessages {
inline void Put32(uint8_t *dst, uint32_t value) {
    for (unsigned i=0; i<4; ++i) dst[i] = uint8_t(value >> (8*i));
}
inline void PutFloat(uint8_t *dst, float value) {
    uint32_t bits; memcpy(&bits, &value, 4); Put32(dst, bits);
}
struct SnapshotBegin {
    static constexpr uint32_t CAN_ID = 0x540;
    uint16_t counter;
};
inline TelemetryFrame Pack(const SnapshotBegin &value) {
    TelemetryFrame frame = {};
    frame.id = SnapshotBegin::CAN_ID;
    frame.data[0] = uint8_t(value.counter);
    frame.data[1] = uint8_t(value.counter >> 8);
    frame.data[2] = 2;
    frame.data[3] = 23;
    return frame;
}

struct SnapshotEnd {
    static constexpr uint32_t CAN_ID = 0x558;
    uint16_t counter;
};
inline TelemetryFrame Pack(const SnapshotEnd &value) {
    TelemetryFrame frame = {};
    frame.id = SnapshotEnd::CAN_ID;
    frame.data[0] = uint8_t(value.counter);
    frame.data[1] = uint8_t(value.counter >> 8);
    frame.data[2] = 2;
    frame.data[3] = 23;
    return frame;
}

struct DriverInputs {
    static constexpr uint32_t CAN_ID = 0x541;
    float APPS_Travel;
    float BSEAvg;
};
inline TelemetryFrame Pack(const DriverInputs &value) {
    TelemetryFrame frame = {};
    frame.id = DriverInputs::CAN_ID;
    PutFloat(frame.data + 0, value.APPS_Travel);
    PutFloat(frame.data + 4, value.BSEAvg);
    return frame;
}

struct BrakePressures {
    static constexpr uint32_t CAN_ID = 0x542;
    float BSEFront;
    float BSERear;
};
inline TelemetryFrame Pack(const BrakePressures &value) {
    TelemetryFrame frame = {};
    frame.id = BrakePressures::CAN_ID;
    PutFloat(frame.data + 0, value.BSEFront);
    PutFloat(frame.data + 4, value.BSERear);
    return frame;
}

struct FrontSuspension {
    static constexpr uint32_t CAN_ID = 0x543;
    float FLLinpots;
    float FRLinpots;
};
inline TelemetryFrame Pack(const FrontSuspension &value) {
    TelemetryFrame frame = {};
    frame.id = FrontSuspension::CAN_ID;
    PutFloat(frame.data + 0, value.FLLinpots);
    PutFloat(frame.data + 4, value.FRLinpots);
    return frame;
}

struct RearSuspension {
    static constexpr uint32_t CAN_ID = 0x544;
    float BLLinpots;
    float BRLinpots;
};
inline TelemetryFrame Pack(const RearSuspension &value) {
    TelemetryFrame frame = {};
    frame.id = RearSuspension::CAN_ID;
    PutFloat(frame.data + 0, value.BLLinpots);
    PutFloat(frame.data + 4, value.BRLinpots);
    return frame;
}

struct BatteryPower {
    static constexpr uint32_t CAN_ID = 0x545;
    float packVoltage;
    float packCurrent;
};
inline TelemetryFrame Pack(const BatteryPower &value) {
    TelemetryFrame frame = {};
    frame.id = BatteryPower::CAN_ID;
    PutFloat(frame.data + 0, value.packVoltage);
    PutFloat(frame.data + 4, value.packCurrent);
    return frame;
}

struct BatteryState {
    static constexpr uint32_t CAN_ID = 0x546;
    float soc;
    float avgCellVolt;
};
inline TelemetryFrame Pack(const BatteryState &value) {
    TelemetryFrame frame = {};
    frame.id = BatteryState::CAN_ID;
    PutFloat(frame.data + 0, value.soc);
    PutFloat(frame.data + 4, value.avgCellVolt);
    return frame;
}

struct BatteryLimits {
    static constexpr uint32_t CAN_ID = 0x547;
    float dischargeLimit;
    float chargeLimit;
};
inline TelemetryFrame Pack(const BatteryLimits &value) {
    TelemetryFrame frame = {};
    frame.id = BatteryLimits::CAN_ID;
    PutFloat(frame.data + 0, value.dischargeLimit);
    PutFloat(frame.data + 4, value.chargeLimit);
    return frame;
}

struct CellVoltages {
    static constexpr uint32_t CAN_ID = 0x548;
    float lowCellVolt;
    float highCellVolt;
};
inline TelemetryFrame Pack(const CellVoltages &value) {
    TelemetryFrame frame = {};
    frame.id = CellVoltages::CAN_ID;
    PutFloat(frame.data + 0, value.lowCellVolt);
    PutFloat(frame.data + 4, value.highCellVolt);
    return frame;
}

struct MotorMotion {
    static constexpr uint32_t CAN_ID = 0x549;
    float eRPM;
    float motorPosition;
};
inline TelemetryFrame Pack(const MotorMotion &value) {
    TelemetryFrame frame = {};
    frame.id = MotorMotion::CAN_ID;
    PutFloat(frame.data + 0, value.eRPM);
    PutFloat(frame.data + 4, value.motorPosition);
    return frame;
}

struct MotorCurrentDemand {
    static constexpr uint32_t CAN_ID = 0x54A;
    float targetIq;
    float acCurrent;
};
inline TelemetryFrame Pack(const MotorCurrentDemand &value) {
    TelemetryFrame frame = {};
    frame.id = MotorCurrentDemand::CAN_ID;
    PutFloat(frame.data + 0, value.targetIq);
    PutFloat(frame.data + 4, value.acCurrent);
    return frame;
}

struct InverterPower {
    static constexpr uint32_t CAN_ID = 0x54B;
    float inputVoltage;
    float dcCurrent;
};
inline TelemetryFrame Pack(const InverterPower &value) {
    TelemetryFrame frame = {};
    frame.id = InverterPower::CAN_ID;
    PutFloat(frame.data + 0, value.inputVoltage);
    PutFloat(frame.data + 4, value.dcCurrent);
    return frame;
}

struct MotorTemperatures {
    static constexpr uint32_t CAN_ID = 0x54C;
    float controllerTemp;
    float motorTemp;
};
inline TelemetryFrame Pack(const MotorTemperatures &value) {
    TelemetryFrame frame = {};
    frame.id = MotorTemperatures::CAN_ID;
    PutFloat(frame.data + 0, value.controllerTemp);
    PutFloat(frame.data + 4, value.motorTemp);
    return frame;
}

struct FOCCurrents {
    static constexpr uint32_t CAN_ID = 0x54D;
    float focId;
    float focIq;
};
inline TelemetryFrame Pack(const FOCCurrents &value) {
    TelemetryFrame frame = {};
    frame.id = FOCCurrents::CAN_ID;
    PutFloat(frame.data + 0, value.focId);
    PutFloat(frame.data + 4, value.focIq);
    return frame;
}

struct ACDriveLimits {
    static constexpr uint32_t CAN_ID = 0x54E;
    float maxAC_Current;
    float avMaxAC_Current;
};
inline TelemetryFrame Pack(const ACDriveLimits &value) {
    TelemetryFrame frame = {};
    frame.id = ACDriveLimits::CAN_ID;
    PutFloat(frame.data + 0, value.maxAC_Current);
    PutFloat(frame.data + 4, value.avMaxAC_Current);
    return frame;
}

struct ACBrakeLimits {
    static constexpr uint32_t CAN_ID = 0x54F;
    float minAC_Current;
    float avMinAC_Current;
};
inline TelemetryFrame Pack(const ACBrakeLimits &value) {
    TelemetryFrame frame = {};
    frame.id = ACBrakeLimits::CAN_ID;
    PutFloat(frame.data + 0, value.minAC_Current);
    PutFloat(frame.data + 4, value.avMinAC_Current);
    return frame;
}

struct DCDriveLimits {
    static constexpr uint32_t CAN_ID = 0x550;
    float maxDC_Current;
    float avMaxDC_Current;
};
inline TelemetryFrame Pack(const DCDriveLimits &value) {
    TelemetryFrame frame = {};
    frame.id = DCDriveLimits::CAN_ID;
    PutFloat(frame.data + 0, value.maxDC_Current);
    PutFloat(frame.data + 4, value.avMaxDC_Current);
    return frame;
}

struct DCBrakeLimits {
    static constexpr uint32_t CAN_ID = 0x551;
    float minDC_Current;
    float avMinDC_Current;
};
inline TelemetryFrame Pack(const DCBrakeLimits &value) {
    TelemetryFrame frame = {};
    frame.id = DCBrakeLimits::CAN_ID;
    PutFloat(frame.data + 0, value.minDC_Current);
    PutFloat(frame.data + 4, value.avMinDC_Current);
    return frame;
}

struct MotorInputs {
    static constexpr uint32_t CAN_ID = 0x552;
    float dti_throttleInput;
    float dti_brakeInput;
};
inline TelemetryFrame Pack(const MotorInputs &value) {
    TelemetryFrame frame = {};
    frame.id = MotorInputs::CAN_ID;
    PutFloat(frame.data + 0, value.dti_throttleInput);
    PutFloat(frame.data + 4, value.dti_brakeInput);
    return frame;
}

struct MotorDutyCycle {
    static constexpr uint32_t CAN_ID = 0x553;
    float dutyCycle;
};
inline TelemetryFrame Pack(const MotorDutyCycle &value) {
    TelemetryFrame frame = {};
    frame.id = MotorDutyCycle::CAN_ID;
    PutFloat(frame.data + 0, value.dutyCycle);
    return frame;
}

struct IsolationResistance {
    static constexpr uint32_t CAN_ID = 0x554;
    float imdResistance;
};
inline TelemetryFrame Pack(const IsolationResistance &value) {
    TelemetryFrame frame = {};
    frame.id = IsolationResistance::CAN_ID;
    PutFloat(frame.data + 0, value.imdResistance);
    return frame;
}

struct VehicleStatus {
    static constexpr uint32_t CAN_ID = 0x555;
    bool RTMState;
    uint8_t controlMode;
    uint8_t isMotorStill;
    uint8_t faultCode;
    uint8_t driveEnabled;
    uint8_t CANmapVersion;
    uint8_t vehicleState;
};
inline TelemetryFrame Pack(const VehicleStatus &value) {
    TelemetryFrame frame = {};
    frame.id = VehicleStatus::CAN_ID;
    frame.data[0] = uint8_t(value.RTMState);
    frame.data[1] = uint8_t(value.controlMode);
    frame.data[2] = uint8_t(value.isMotorStill);
    frame.data[3] = uint8_t(value.faultCode);
    frame.data[4] = uint8_t(value.driveEnabled);
    frame.data[5] = uint8_t(value.CANmapVersion);
    frame.data[6] = uint8_t(value.vehicleState);
    return frame;
}

struct VehicleFlags {
    static constexpr uint32_t CAN_ID = 0x556;
    bool digitalIn1;
    bool digitalIn2;
    bool digitalIn3;
    bool digitalIn4;
    bool digitalOut1;
    bool digitalOut2;
    bool digitalOut3;
    bool digitalOut4;
    bool capTempLimitActive;
    bool dcTempLimitActive;
    bool driveEnableLimitActive;
    bool IGBTaccelLimitActive;
    bool IGBTtempLimitActive;
    bool inputVoltageLimitActive;
    bool motorAccelTempLimitActive;
    bool motorTempLimitActive;
    bool RPMminLimitActive;
    bool RPMmaxLimitActive;
    bool powerLimitActive;
    uint32_t imdStatus;
};
inline TelemetryFrame Pack(const VehicleFlags &value) {
    TelemetryFrame frame = {};
    frame.id = VehicleFlags::CAN_ID;
    frame.data[0] |= uint8_t(value.digitalIn1) << 0;
    frame.data[0] |= uint8_t(value.digitalIn2) << 1;
    frame.data[0] |= uint8_t(value.digitalIn3) << 2;
    frame.data[0] |= uint8_t(value.digitalIn4) << 3;
    frame.data[0] |= uint8_t(value.digitalOut1) << 4;
    frame.data[0] |= uint8_t(value.digitalOut2) << 5;
    frame.data[0] |= uint8_t(value.digitalOut3) << 6;
    frame.data[0] |= uint8_t(value.digitalOut4) << 7;
    frame.data[1] |= uint8_t(value.capTempLimitActive) << 0;
    frame.data[1] |= uint8_t(value.dcTempLimitActive) << 1;
    frame.data[1] |= uint8_t(value.driveEnableLimitActive) << 2;
    frame.data[1] |= uint8_t(value.IGBTaccelLimitActive) << 3;
    frame.data[1] |= uint8_t(value.IGBTtempLimitActive) << 4;
    frame.data[1] |= uint8_t(value.inputVoltageLimitActive) << 5;
    frame.data[1] |= uint8_t(value.motorAccelTempLimitActive) << 6;
    frame.data[1] |= uint8_t(value.motorTempLimitActive) << 7;
    frame.data[2] |= uint8_t(value.RPMminLimitActive) << 0;
    frame.data[2] |= uint8_t(value.RPMmaxLimitActive) << 1;
    frame.data[2] |= uint8_t(value.powerLimitActive) << 2;
    Put32(frame.data + 4, value.imdStatus);
    return frame;
}

struct VehicleFaults {
    static constexpr uint32_t CAN_ID = 0x557;
    uint32_t faultMap;
};
inline TelemetryFrame Pack(const VehicleFaults &value) {
    TelemetryFrame frame = {};
    frame.id = VehicleFaults::CAN_ID;
    Put32(frame.data, value.faultMap);
    return frame;
}

} // namespace TelemetryMessages
