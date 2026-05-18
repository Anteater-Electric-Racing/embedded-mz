use std::any::TypeId;
use std::fmt::Debug;
use config::Map;
use rumqttc::tokio_rustls::rustls::crypto::cipher::InboundOpaqueMessage;
use serde::{Serialize, Deserialize};
use rumqttc::{AsyncClient, ClientError, MqttOptions, QoS};
use serde_json::map::Values;
use tracing::field::display;
use tracing::{error, info};
use tokio::sync::{Mutex, OnceCell};
use tokio::time::Duration;
use questdb::{
    Result,
    ingress::{
        Sender,
        SenderBuilder,
        TimestampMicros}};
use crate::can::MCUWarningLevel;

pub const TAOS_URL: &str = "taos+ws://localhost:6041/fsae";
pub const MQTT_ID: &str = "fsae";
pub const MQTT_HOST: &str = "127.0.0.1";
pub const MQTT_PORT: u16 = 1883;
pub const QUESTDB_URL: &str = "http::addr=localhost:9000;"; //this is probably (defintly) the wrong link but it does let me connect on my home computer


const CREATE_DB: &str =
    "CREATE DATABASE IF NOT EXISTS fsae WAL_LEVEL 2 WAL_FSYNC_PERIOD 0 STT_TRIGGER 1 KEEP 365d";
const MAX_CONSECUTIVE_FAILURES: u32 = 10;
const RECONNECT_DELAY: Duration = Duration::from_secs(2);

pub trait Reading: Serialize {
    fn topic() -> &'static str;
}

//static TDENGINE: OnceCell<Sender<String>> = OnceCell::const_new();
static MQTT_CLIENT: OnceCell<AsyncClient> = OnceCell::const_new();
static QUESTDB: OnceCell<Sender> = OnceCell::const_new();

struct Sending_data { //TODO: rename
    pub buffer : questdb::ingress::Buffer
}

impl Sending_data {
    async fn send_questdb(&mut self){
        /*info!("{}", match self.buffer.check_can_flush() {
            Ok(_val) => "ok",
            Err(_val) => "err"
        });*/
        let mut sender: Sender =  get_questdb_sender().await;
        let _ = sender.flush(&mut self.buffer);
    }
}

async fn get_questdb_sender() -> Sender{
    loop {
        match Sender::from_conf(QUESTDB_URL) {
            Ok(t) => return t,
            Err(e) => {
                error!(%e, "Failed to connect to QuestDB, retrying in {RECONNECT_DELAY:?}");
                tokio::time::sleep(RECONNECT_DELAY).await;
            }
        }
    }
}

#[derive(Deserialize, PartialEq, Debug)]
#[serde(untagged)]
enum PosssibleFields {
    Int(i64),
    Float(f32),
    Bool(bool),
    MCUWarningLevel(MCUWarningLevel)
}

/*fn mapify(a: impl Serialize) -> HashMap<String, StringOrI32OrF32> {
    let val = serde_json::to_value(a).unwrap();
    serde_json::from_value(val).unwrap()
} */

 
async fn data_to_buffer(table_name: &str, value: serde_json::Value) -> questdb::ingress::Buffer{
    let mut buf = Sender::from_conf("http::addr=localhost:9000;").expect("lol").new_buffer(); 
    let _ = buf.table("telemetry");
    let as_map : Map<String, PosssibleFields> = serde_json::from_value(value.clone()).unwrap();
    let as_map2 : Map<String, PosssibleFields> = serde_json::from_value(value).unwrap();

    for (key, value) in as_map.into_iter(){
        if let PosssibleFields::Int(f) = value {
            let _ = buf.column_i64(key.as_str(), f);
        }
        else if let PosssibleFields::Float(f) = value {
            let _ = buf.column_f64(key.as_str(), f.into());
        }
        else if let PosssibleFields::Bool(f) = value {
            let _ = buf.column_bool(key.as_str(), f);
        }
        /*else if let PosssibleFields::MCUWarningLevel(f) = value {
            let _ = buf.column_str(key.as_str(), f.into());
        }*/
    }

    let _ = buf.column_str("col_name", "value");
    if let PosssibleFields::Int(i) = as_map2.get("ts").expect("msg"){
        let _ = buf.at(TimestampMicros::new((*i).into()));
    }
    buf

}

async fn get_mqtt_client() -> &'static AsyncClient {
    MQTT_CLIENT
        .get_or_init(|| async {
            let opts = MqttOptions::new(MQTT_ID, MQTT_HOST, MQTT_PORT);
            let (client, mut eventloop) = AsyncClient::new(opts, 100_000);
            tokio::spawn(async move {
                loop {
                    if let Err(e) = eventloop.poll().await {
                        error!(%e, "MQTT eventloop error");
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
            });
            client
        })
        .await
}

#[inline]
fn push_field(buf: &mut String, k: &str, v: &serde_json::Value) {
    buf.push_str(k);
    buf.push('=');
    match v {
        serde_json::Value::Bool(b) => {
            buf.push_str(if *b { "true" } else { "false" });
        }
        serde_json::Value::Number(n) => {
            if let Some(f) = n.as_f64() {
                buf.push_str(ryu::Buffer::new().format(f));
                buf.push_str("f32");
            } else if let Some(i) = n.as_i64() {
                buf.push_str(itoa::Buffer::new().format(i));
                buf.push_str("i32");
            }
        }
        other => {
            buf.push('"');
            buf.push_str(&other.to_string());
            buf.push('"');
        }
    }
}

fn to_line_protocol_from_value(
    measurement: &str,
    map: &serde_json::Value,
    timestamp_ms: u64,
) -> Option<String> {
    let obj = map.as_object()?;
    let mut buf = String::with_capacity(measurement.len() + 1 + obj.len() * 30 + 20);
    buf.push_str(measurement);
    buf.push(' ');

    let mut iter = obj.iter();
    if let Some((k, v)) = iter.next() {
        push_field(&mut buf, k, v);
    }
    for (k, v) in iter {
        buf.push(',');
        push_field(&mut buf, k, v);
    }

    buf.push(' ');
    buf.push_str(itoa::Buffer::new().format(timestamp_ms));

    Some(buf)
}

pub fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock went backwards")
        .as_millis() as u64
}

pub async fn send_message<T: Reading + Send + 'static>(message: T, timestamp_ms: u64) {
    let mut value: serde_json::Value = match serde_json::to_value(&message) {
        Ok(v) => v,
        Err(e) => {
            error!(%e, "Failed to serialize message");
            return;
        }
    };
    //info!("{}", value);

    if let Some(obj) = value.as_object_mut() {
        obj.insert("ts".to_string(), serde_json::json!(timestamp_ms));
    }// inserts time

    

    let json = value.to_string(); //data as string
    let topic = T::topic(); //name of struct basically
    

    match get_mqtt_client()
        .await
        .try_publish(topic, QoS::AtMostOnce, false, json)
    {
        Ok(()) => {}
        Err(ClientError::TryRequest(_)) => {
            tracing::warn!("MQTT channel full — dropping message");
        }
        Err(e) => error!(%e, "MQTT publish error"),
    }

    
    let buf = data_to_buffer( topic, value).await;
    let mut data = Sending_data {buffer : buf};
    data.send_questdb().await;

}
