//! This module enables the receiving and decoding of telemetry
//! data from the motor controller over CAN ISO-TP and sets up test
//! functions for transmitting test packets over vCAN.
//!
//! Module includes:
//!
//! - ISO-TP socket setup for receiving and sending frames
//! - Parsing of the MCU telemetry packet into typed data
//! - Definitions of all enums representing MCU and motor state machines
//! - `TelemetryData` structure, which contains the fully decoded packet
//! - Test function to send and verify dummy telemetry over vcan0
//!
//! Production code relies on `can0`, while tests can run against vcan0
//! using the virtual CAN network setup in the GitHub Actions workflow (test.yml).

use structmap::{ToMap, value::Value};
use crate::send::{now_ms, send_message, Reading, get_qdb_buffer, get_questdb_sender};
use questdb::ingress::{
    Sender,
    Buffer
};
use deku::prelude::*;
use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::time::Duration;
use tokio::time::sleep;
use tokio_socketcan_isotp::{IsoTpSocket, StandardId};
use tracing::{error, info, warn};

const CAN_INTERFACE: &str = "can0";
const CAN_SRC_ID: u16 = 0x666;
const CAN_DST_ID: u16 = 0x777;

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum MotorState {
    #[deku(id = 0)]
    #[default]
    Off,
    #[deku(id = 1)]
    Precharging,
    #[deku(id = 2)]
    Idle,
    #[deku(id = 3)]
    Driving,
    #[deku(id = 4)]
    Fault,
}

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum MotorRotateDirection {
    #[deku(id = 0)]
    #[default]
    Standby,
    #[deku(id = 1)]
    Forward,
    #[deku(id = 2)]
    Backward,
    #[deku(id = 3)]
    Error,
}

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum MCUMainState {
    #[deku(id = 0)]
    #[default]
    Off,
    #[deku(id = 1)]
    Precharging,
    #[deku(id = 2)]
    Idle,
    #[deku(id = 3)]
    Driving,
    #[deku(id = 4)]
    Fault,
}

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum VehicleState {
    #[deku(id = 0)]
    #[default]
    Off,
    #[deku(id = 1)]
    Precharging,
    #[deku(id = 2)]
    Idle,
    #[deku(id = 3)]
    Driving,
    #[deku(id = 4)]
    Fault,
}

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum MCUWorkMode {
    #[deku(id = 0)]
    #[default]
    Standby,
    #[deku(id = 1)]
    Torque,
    #[deku(id = 2)]
    Speed,
}

#[derive(
    Default,
    Debug,
    Deserialize_repr,
    Serialize_repr,
    PartialEq,
    Clone,
    Copy,
    DekuRead,
    DekuWrite,
    DekuSize,
)]
#[deku(ctx = "endian: deku::ctx::Endian")]
#[deku(id_type = "u8")]
#[repr(u8)]
pub enum MCUWarningLevel {
    #[deku(id = 0)]
    #[default]
    None,
    #[deku(id = 1)]
    Low,
    #[deku(id = 2)]
    Medium,
    #[deku(id = 3)]
    High,
}

/// Telemetry data record produced by the motor controller.
///
/// Parsed from the CAN_PACKET_SIZE-byte ISO-TP frame received over CAN.
/// Contains driver inputs, motor state information, controller status,
/// temperatures, electrical measurements, fault flags, and debug channels.
#[derive(
    Serialize, Deserialize, Default, Debug, Clone, PartialEq, DekuRead, DekuWrite, DekuSize,
)]
#[deku(endian = "little")]pub struct TelemetryData {
    pub apps_travel: f32,

    pub bse_front: f32,
    pub bse_rear: f32,

    pub imd_resistance: f32,
    pub imd_status: u32,

    pub pack_voltage: f32,
    pub pack_current: f32,
    pub soc: f32,
    pub discharge_limit: f32,
    pub charge_limit: f32,
    pub low_cell_volt: f32,
    pub high_cell_volt: f32,
    pub avg_cell_volt: f32,

    pub motor_speed: f32,
    pub motor_torque: f32,
    pub max_motor_torque: f32,
    pub motor_direction: MotorRotateDirection,
    pub motor_state: MotorState,

    pub mcu_main_state: MCUMainState,
    pub mcu_work_mode: MCUWorkMode,

    pub mcu_voltage: f32,
    pub mcu_phase_current: f32,
    pub mcu_current: f32,

    pub motor_temp: i32,
    pub mcu_temp: i32,

    pub mcu_warning_level: MCUWarningLevel,

    pub shocktravel1: f32,
    pub shocktravel2: f32,
    pub shocktravel3: f32,
    pub shocktravel4: f32,

    pub dc_main_wire_over_volt_fault: bool,
    pub motor_phase_curr_fault: bool,
    pub mcu_over_hot_fault: bool,
    pub resolver_fault: bool,
    pub phase_curr_sensor_fault: bool,
    pub motor_over_spd_fault: bool,
    pub drv_motor_over_hot_fault: bool,
    pub dc_main_wire_over_curr_fault: bool,
    pub drv_motor_over_cool_fault: bool,
    pub dc_low_volt_warning: bool,
    pub mcu_12v_low_volt_warning: bool,
    pub motor_stall_fault: bool,
    pub motor_open_phase_fault: bool,

    #[deku(bits = 1)]
    pub over_current: bool,
    #[deku(bits = 1)]
    pub under_voltage: bool,
    #[deku(bits = 1)]
    pub over_temperature: bool,
    #[deku(bits = 1)]
    pub apps: bool,
    #[deku(bits = 1)]
    pub bse: bool,
    #[deku(bits = 1)]
    pub bpps: bool,
    #[deku(bits = 1)]
    pub apps_brake_plaus: bool,
    #[deku(bits = 1, pad_bits_after = "24")]
    pub low_battery_voltage: bool,
}

impl Reading for TelemetryData {
    fn topic() -> &'static str {
        "telemetry"
    }
}

#[derive(
    Serialize, Deserialize, Default, Debug, Clone, PartialEq, DekuRead, DekuWrite, DekuSize,
)]
#[deku(endian = "little")]pub struct TelemetryData2 {
    pub apps_travel: f32,

    pub bse_front: f32,
    pub bse_rear: f32,
    pub bse_avg: f32,

    pub imd_resistance: f32,
    pub imd_status: u32,

    pub pack_voltage: f32,
    pub pack_current: f32,
    pub soc: f32,
    pub discharge_limit: f32,
    pub charge_limit: f32,
    pub low_cell_volt: f32,
    pub high_cell_volt: f32,
    pub avg_cell_volt: f32,

    //MCU data
    pub control_mode: u8,
    pub target_iq: f32,

    pub motor_position: f32,
    pub is_motor_still: u8,
    pub e_rpm: f32,
    pub duty_cycle: f32,
    pub input_voltage: f32,
    pub ac_current: f32,
    pub dc_current: f32,
    pub controller_temp: f32,
    pub motor_temp: f32,
    pub fault_code: u8,
    pub foc_id: f32,
    pub foc_iq: f32,
    
    pub drive_enabled: u8,

    pub max_ac_current: f32,
    pub av_max_ac_current: f32,
    pub min_ac_current: f32,
    pub av_min_ac_current: f32,
    pub max_dc_current: f32,
    pub av_max_dc_current: f32,
    pub min_dc_current: f32,
    pub av_min_dc_current: f32,

    pub dti_throttle_input: f32,
    pub dti_brake_input: f32,

    pub digital_in1: bool,
    pub digital_in2: bool,
    pub digital_in3: bool,
    pub digital_in4: bool,
    pub digital_out1: bool,
    pub digital_out2: bool,
    pub digital_out3: bool,
    pub digital_out4: bool,

    // Limits Active - Group 1
    pub cap_temp_limit_active: bool,
    pub dc_temp_limit_active: bool,
    pub drive_enable_limit_active: bool,
    pub igbt_accel_limit_active: bool,
    pub igbt_temp_limit_active: bool,
    pub input_voltage_limit_active: bool,
    pub motor_accel_temp_limit_active: bool,
    pub motor_temp_limit_active: bool,

    // Limits Active - Group 2
    pub rpm_min_limit_active: bool,
    pub rpm_max_limit_active: bool,
    pub power_limit_active: bool,
    
    pub can_nmap_version: u8,

    pub vehicle_state: u16,

    #[deku(bits = 1)]
    pub osr_current: bool,
    #[deku(bits = 1)]
    pub under_voltage: bool,
    #[deku(bits = 1)]
    pub over_temperature: bool,
    #[deku(bits = 1)]
    pub apps: bool,
    #[deku(bits = 1)]
    pub bse: bool,
    #[deku(bits = 1)]
    pub bpps: bool,
    #[deku(bits = 1)]
    pub apps_brake_plaus: bool,
    #[deku(bits = 1, pad_bits_after = "24")]
    pub low_battery_voltage: bool,
}

impl Reading for TelemetryData2 {
    fn topic() -> &'static str {
        "July_19_testing"
    }
}


//I'm very sorry but I honestly can't figure out how to properly have a global & mutable buffer (which rust kinda doesn't want to have (no shared mutable states)), so I;m doing some architectural ersosion and putting buffer here. 

/// Reads ISO-TP packets from `can0` in a loop, parses each into
/// [`TelemetryData`], and forwards via [`send_message`].
///
/// Retries socket creation on failure; logs malformed packets.
async fn read_can_hardware() {
    let mut buffer : Buffer = get_qdb_buffer();
    let mut sender : Sender = get_questdb_sender().await;
    loop {
        let socket = match IsoTpSocket::open(
            CAN_INTERFACE,
            StandardId::new(CAN_SRC_ID).expect("Invalid src id"),
            StandardId::new(CAN_DST_ID).expect("Invalid dst id"),
        ) {
            Ok(socket) => socket,
            Err(e) => {
                error!(%e, "Failed to open CAN socket");
                sleep(Duration::from_secs(1)).await;
                continue;
            }
        };

        while let Ok(packet) = socket.read_packet().await {
            let ts = now_ms();
            match TelemetryData2::from_bytes((packet.as_ref(), 0)) {
                Ok(((remaining, _), _)) if !remaining.is_empty() => {
                    warn!("Telemetry packet has {} trailing bytes", remaining.len(),);
                }
                Ok((_, data)) => send_message(data, ts, &mut buffer, &mut sender).await,
                Err(e) => warn!(error = %e, "Malformed telemetry packet"),
            }
        }
    }
}

impl TelemetryData2 {
    fn mutate_test_val(&mut self) -> TelemetryData2{
        self.imd_status=now_ms() as u32;
        self.clone()
    }
}

/// Generates synthetic telemetry (debug builds only).
async fn read_can_synthetic() {
    let mut buffer : Buffer = get_qdb_buffer();
    let mut sender : Sender = get_questdb_sender().await;
    use std::time::Instant;

    let mut count: u64 = 0;
    let mut last = Instant::now();

    let mut interval = tokio::time::interval(Duration::from_millis(1));
    loop {
        interval.tick().await;
        send_message(TelemetryData2::default().mutate_test_val(), now_ms(), &mut buffer, &mut sender).await;
        count += 1;

        let elapsed = last.elapsed();
        if elapsed >= Duration::from_secs(1) {
            info!("{:.0} msg/s", count as f64 / elapsed.as_secs_f64());
            count = 0;
            last = Instant::now();
        }
    }
}


/// Entry point: dispatches to the hardware or synthetic reader depending
/// on the build profile.
pub async fn read_can() {
    if cfg!(feature = "synthetic") {
        read_can_synthetic().await;
    } else {
        read_can_hardware().await;
    }
}
