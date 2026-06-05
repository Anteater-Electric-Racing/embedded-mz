from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1351
from time import sleep
import paho.mqtt.subscribe as subscribe



#Ingest data
#Process
#   Extract tag
#   Extract_warning
#Display

RED = "RED"
GREEN = "GREEN"
SELECTED = "pack_current"

def parse_args():
    #planned flags:
    # -h help
    # --select what column
    # --warn [none/warn/fault] look for faults or faults + warns or ignore faults and wanrs 
    print("parse_args stub")


int main:
    parse_args()
    
    while True: 
        mqtt_msg = ingest_mqtt();
        if (has_warning(mqtt_msg)):
            warnings = extract_warnings(mqtt_msg)
            for warn in warning:
                display(warn, RED, delay = 100)
        
        selected_data = extract_column(mqtt_msg, GREEN) 

#serial = spi(device = 0, port = 0)

#device  = ssd1351(serial)
#with canvas(device) as draw:
#    draw.text((30, 40), "please work", fill = "white")
#sleep(1000)
