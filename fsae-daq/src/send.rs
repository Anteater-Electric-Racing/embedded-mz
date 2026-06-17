use std::any::TypeId;
use std::default;
use std::fmt::Debug;
use std::sync::LazyLock;
use config::Map;
use serde::{Serialize, Deserialize};
use rumqttc::{AsyncClient, ClientError, MqttOptions, QoS};
use serde_json::map::Values;
use tracing::field::display;
use lazy_static::lazy_static;
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

lazy_static! {
    static ref Questdb_buffer : Mutex<questdb::ingress::Buffer> = 
    Mutex::new(get_qdb_buffer());
    static ref loss: Mutex<i32> = Mutex::new(10);

}
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

pub async fn get_questdb_sender() -> Sender{
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
    Bool(bool)
}

pub fn get_qdb_buffer() -> questdb::ingress::Buffer{
    Sender::from_conf("http::addr=localhost:9000;").expect("lol").new_buffer()
}

async fn data_into_buffer(table_name: &str, value: serde_json::Value, buffer : &mut questdb::ingress::Buffer){
    let _ = buffer.table(table_name);
    let as_map : Map<String, PosssibleFields> = serde_json::from_value(value.clone()).unwrap();
    let as_map2 : Map<String, PosssibleFields> = serde_json::from_value(value).unwrap();

    for (key, value) in as_map.into_iter(){
        if let PosssibleFields::Int(f) = value {
            let _ = buffer.column_i64(key.as_str(), f);
        }
        else if let PosssibleFields::Float(f) = value {
            let _ = buffer.column_f64(key.as_str(), f.into());
        }
        else if let PosssibleFields::Bool(f) = value {
            let _ = buffer.column_bool(key.as_str(), f);
        }
    }

    let _ = buffer.column_str("col_name", "value");
    if let PosssibleFields::Int(i) = as_map2.get("ts").expect("msg"){
        let _ = buffer.at(TimestampMicros::new((*i)*1000));
    }
}

async fn get_mqtt_client() -> &'static AsyncClient {
    MQTT_CLIENT
        .get_or_init(|| async {
            let opts = MqttOptions::new(MQTT_ID, MQTT_HOST, MQTT_PORT);
            let (client, mut eventloop) = AsyncClient::new(opts, 1_000);
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

pub fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock went backwards")
        .as_millis() as u64
}

pub async fn send_message<T: Reading + Send + 'static>(message: T, timestamp_ms: u64, buffer : &mut questdb::ingress::Buffer, sender : &mut Sender) {
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
    
    match *(*loss).lock().await {
        10 => {
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
            let mut lock = (*loss).lock().await;
            *lock -= 10;
        }
        _ => {
            let mut lock = (*loss).lock().await;
            *lock -= 1;
        }
    }
    
    
    //async fn send_to_questdb(topic : &str, value : serde_json::Value) {
    //    let buf = data_to_buffer( topic, value).await;
    //    let mut data = Sending_data {buffer : buf};
    //    data.send_questdb().await;
    //}

    //put into buffer
    data_into_buffer(topic, value, buffer).await;
    //info!("what");
    //info!("sent to qdb {}", buffer.row_count());
    //check size
    if buffer.row_count() > 1_000 {
        //send to buffer if fast enough
        let _ = sender.flush( buffer);
        //info!("sent to qdb {}", buffer.row_count());
    }
    //tokio::time::sleep(Duration::from_millis(1)).await; //debugging cursor start with slower ingress?

    //tokio::spawn(send_to_questdb(topic, value));
    //let buf = data_to_buffer( topic, value).await;
    //let mut data = Sending_data {buffer : buf};
    //data.send_questdb().await;

}
