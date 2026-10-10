# Telemetry handoff — 2026-10-09

Read this before resuming telemetry work. This records completed work and known
limitations; it does not authorize committing, pushing, discarding changes, or
accessing external services.

## User scope and Git state

- User restricts general file access/work to `embedded-mz` and prohibits internet
  or external app/service access. Specific exceptions authorized: put Python/test
  artifacts in `/private/tmp/embedded-mz-can-artifacts`, and copy fake traces to
  `/Users/anoop.kuganti/Downloads`. Do not generalize these exceptions.
- Current branch: `anoop/telem-off-isotp`, renamed from
  `codex/named-can-telemetry`. Created from **local main**, commit
  `09af0e952dd46df1b3de0dc3ac89fc40193ed1db`; no remote refresh was performed.
- Work is **uncommitted**, with both staged and unstaged changes. The user declined
  the commit approval. In particular, the dual-bus changes are newer than the
  staged versions. Do not assume `git diff --cached` contains the final work.
- Earlier prototype, including a Rust DAQ migration, is preserved in a stash:
  `Preserve CAN prototype before main-based named telemetry branch`, created on
  `anoop/MZ_Finale`. Locate by message; do not blindly apply `stash@{0}` later.
- As of this note, `.pio` build files/submodule and `speaker.cpp` also have changes.
  These are outside the telemetry change; preserve them and exclude from telemetry
  staging unless the user requests otherwise.
- User reports pull/push unavailable and wants to resume later. No diagnosis,
  pull, push, or remote authentication action was attempted for this request.

## Completed C++ and DBC work

- `fsae-ccm/src/vehicle/comms/telemetry.cpp`: samples current readings and constructs
  individual named structs with `Pack(...)` calls, then sends each frame on both
  **CAN2 and CAN3**, configured at **500 kbit/s**.
- `fsae-ccm/src/vehicle/comms/telemetry_frames.h`: named message structs, explicit
  little-endian packing, IEEE754 binary32 floats. No raw struct memcpy onto CAN;
  padding/bitfield/enum ABI does not determine the wire layout.
- `fsae-ccm/src/vehicle/comms/telemetry.h`: 66-field sample model with DTI readings,
  driver inputs, suspension, BMS, IMD, vehicle state and fault map.
- `fsae-ccm/src/peripherals/can.cpp` and `.h`: removed ISO-TP sender/setup;
  `bool CAN_SendFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length)`
  selects bus 2 or 3, validates inputs, builds a fresh standard CAN frame, and
  reports enqueue success. Other send overloads also use fresh local frames to
  prevent stale DLC/extended flags. One-byte DTI commands avoid reading an
  uninitialized bitfield flag; byte copy respects DLC.
- Main's DTI API uses `targetLq`, `focLd`, `focLq`; sender maps those getters into
  telemetry fields `targetIq`, `focId`, `focIq`. Do not copy the older working
  branch's getter spelling without checking main.
- `can/embedded-mz.dbc`: 64 message definitions, 301 signals including existing
  device definitions and the new named telemetry. Float signals use
  `SIG_VALTYPE_ ... : 1`. CANtaloupe must support this declaration.
- No Python scripts or fake traces remain in the repo's `can/` directory; it
  contains only the DBC.

## Wire layout: named telemetry protocol v2

Every message is standard Classic CAN, DLC 8. Snapshot period is **50 ms (20 Hz)**,
changed from the original 10 ms. No ISO-TP or CAN FD needed. All 38 floats retain
their full precision. Two floats per message except the two singleton readings.

| ID | Name | Fields |
| --- | --- | --- |
| 0x540 | SnapshotBegin | uint16 counter, version=2, data count=23, reserved zeros |
| 0x541 | DriverInputs | APPS_Travel, BSEAvg |
| 0x542 | BrakePressures | BSEFront, BSERear |
| 0x543 | FrontSuspension | FLLinpots, FRLinpots |
| 0x544 | RearSuspension | BLLinpots, BRLinpots |
| 0x545 | BatteryPower | packVoltage, packCurrent |
| 0x546 | BatteryState | soc, avgCellVolt |
| 0x547 | BatteryLimits | dischargeLimit, chargeLimit |
| 0x548 | CellVoltages | lowCellVolt, highCellVolt |
| 0x549 | MotorMotion | eRPM, motorPosition |
| 0x54A | MotorCurrentDemand | targetIq, acCurrent |
| 0x54B | InverterPower | inputVoltage, dcCurrent |
| 0x54C | MotorTemperatures | controllerTemp, motorTemp |
| 0x54D | FOCCurrents | focId, focIq |
| 0x54E | ACDriveLimits | maxAC_Current, avMaxAC_Current |
| 0x54F | ACBrakeLimits | minAC_Current, avMinAC_Current |
| 0x550 | DCDriveLimits | maxDC_Current, avMaxDC_Current |
| 0x551 | DCBrakeLimits | minDC_Current, avMinDC_Current |
| 0x552 | MotorInputs | dti_throttleInput, dti_brakeInput |
| 0x553 | MotorDutyCycle | dutyCycle, four reserved bytes |
| 0x554 | IsolationResistance | imdResistance, four reserved bytes |
| 0x555 | VehicleStatus | RTMState, controlMode, isMotorStill, faultCode, driveEnabled, CANmapVersion, vehicleState, reserved: one byte each |
| 0x556 | VehicleFlags | digital bits 0–7, limit bits 8–18, reserved bits 19–31; uint32 imdStatus in bytes 4–7 |
| 0x557 | VehicleFaults | full uint32 faultMap, four reserved bytes |
| 0x558 | SnapshotEnd | matching counter/version/count, reserved zeros |

Counter wraps at 65535. Fault bits 0–8: overcurrent, undervoltage, overtemperature,
APPS, BSE, BPPS, APPS/brake plausibility, CAN timeout, low battery voltage. Remaining
bits are preserved as Fault_UnknownBits. `vehicleState` uses `VCU_GetState()`.

The sender tracks active and pending buses per frame. Once a bus accepts a frame,
it is not retried there. After a shared 20 ms retry budget, only buses still
pending abort their snapshot; healthy buses can finish. An aborted bus omits the
footer. A matching footer identifies completion, not guaranteed physical delivery.
Telemetry adds 500 frames/s, roughly 11–14% load **per bus**, excluding errors and
retransmissions. Existing CAN2 utilization and control latency need measurement.

## Test artifacts outside the repo

- Primary ASC trace:
  `/private/tmp/embedded-mz-can-artifacts/synthetic-telemetry-classic.asc`
- Candump equivalent:
  `/private/tmp/embedded-mz-can-artifacts/synthetic-telemetry-classic.log`
- Copies delivered to Downloads:
  `/Users/anoop.kuganti/Downloads/synthetic-telemetry-classic.asc` and `.log`.
- Trace is 30 seconds, 15,025 frames, 25 standard IDs on one logical channel.
  Precharge, acceleration, cruise, regeneration, stop; APPS/CAN timeout fault bits
  toggle at 28–29 seconds. It does not duplicate frames for two physical buses.
- Python specification/generation/validation files are in that temporary artifact
  folder. They reference this repo explicitly. The DBC generator writes there;
  copy its output to `can/embedded-mz.dbc` after regenerating.
- Named C++ generator also writes the sender's Pack array/header. Review before
  regenerating: hand edits outside its generated block are preserved, but its
  template should not overwrite newer wire-layout choices unnoticed.
- Temporary files may be cleaned by macOS. Prefer the Downloads copies for traces;
  source of truth for the implemented protocol is the C++ and DBC in the repo.

## Validation and remaining work

- DBC bounds/overlap/ID/float declarations and 18 known payload vectors passed.
- Compiled the actual named C++ sender Pack array with a host compiler and matched
  every encoded byte for three exhaustive fixtures exercising all 66 fields.
- All 15,025 saved ASC/candump trace frames and their decoded DBC signals passed.
- Host tests using the actual dual-bus helper/retry loop passed: both buses,
  transient failures without duplicates, either bus permanently blocked, mid-set
  congestion, both buses blocked, bounded timeout and invalid inputs. Harness is
  outside repo at `.../.build/test_dual_bus.cpp`.
- Full Teensy build, flashing, physical bus behavior and CANtaloupe's successful
  decoding of this final version remain unverified. Do not describe host tests as
  hardware validation.
- **DAQ is unchanged on this branch** and still uses the old ISO-TP receiver. It
  needs a new protocol-v2 decoder to log these frames. Earlier seven Rust decoder
  tests were for the stashed protocol-v1 prototype, not this final branch/version.
- Pre-existing main BMS issue: OBMS1 declares int32 packCurrent despite a two-byte
  comment, whereas initial DBC assumes intended int16 layout. Not fixed here;
  verify Orion configuration rather than assuming firmware/DBC agreement.
- DTI/PCC/IMD/charger mappings retain original code-derived/provisional assumptions.
  Hardware/vendor layouts and checksum behavior need confirmation.

## Future ESP32 migration context (discussion only)

User plans ESP32-S3-WROOM-1. Its built-in TWAI is one Classic CAN controller and
requires a transceiver; it cannot directly replace two independent Teensy buses
or natively transmit CAN-FD. An external controller can provide a second bus;
an FD-capable controller plus compatible transceiver enables CAN-FD, usually over
SPI. Controller model is not selected. No ESP32/FD code or hardware changes made.
Current float message layouts remain portable; FlexCAN_T4 backend would be replaced.

## Fast resume

1. Read this note, current Git status, and staged plus unstaged telemetry diffs.
2. Keep the user's other edits and the saved prototype intact.
3. For CANtaloupe, use the **named protocol-v2** DBC and classic trace, not the
   older reassembled-ISO-TP DBC/trace or numbered protocol-v1 groups.
4. Follow the user's next instruction; do not retry declined commits or initiate
   network operations based on this handoff alone.
