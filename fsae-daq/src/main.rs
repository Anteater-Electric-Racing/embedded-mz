mod can;
mod mqtt;
mod send;
#[cfg(test)]
mod test;

use can::read_can;
use mqtt::mqttd;

use questdb::{
    ingress::{
        Sender,
        TimestampNanos}};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    //tracing_subscriber::fmt()
    //    .pretty()
    //    .with_env_filter("info")
    //    .init();

    //tokio::spawn(read_can());

    //mqttd();

    println!("mow");
    send_info();

    Ok(())
}

pub fn send_info() -> questdb::Result<()> {
   let mut sender = Sender::from_conf("http::addr=host.docker.internal:9000;")?;
   let mut buffer = sender.new_buffer();
   buffer
       .table("trades")?
       .symbol("symbol", "ETH")?
       .symbol("side", "sell")?
       .column_f64("price", 2615.54)?
       .column_f64("amount", 0.00044)?
       .at(TimestampNanos::now())?;
   sender.flush(&mut buffer)?;
   Ok(())
}
