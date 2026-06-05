from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1351
from time import sleep
import paho.mqtt.subscribe as subscribe
import json


#Ingest data
#Process
#   Extract tag
#   Extract_warning
#Display

RED = "RED"
GREEN = "GREEN"
SELECTED = "pack_current"
TOPIC = "telemetry"
HOST = "127.0.0.1"
SUBSCRIPTION = ""

def parse_args():
    global SUBSCRIPTION
    #planned flags:
    # -h help
    # --select what column
    # --warn [none/warn/fault] look for faults or faults + warns or ignore faults and wanrs 
    print("parse_args stub")
    SUBSCRIPTION = subscribe.simple(TOPIC, hostname=HOST)
    #this function will also have to reintialize the subscriber but anyways

def tests():
    #very scuffed
    parse_args()
    print(mqtt_msg := ingest_mqtt())
    print(type(mqtt_msg), " should be dict")
    print(has_fault({"motor_fault": True}), " should be True (has_fault)")
    print(has_fault({"motor_fault": False}), " should be False (has_fault)")

def ingest_mqtt():
   payload = json.loads(SUBSCRIPTION.payload)
   return payload

def has_fault(payload : dict):
    for key, value in payload.items():
        if "fault" in key and value == True:
            return True
    return False

def main():
    parse_args()
    

"""
    while True: 
        mqtt_msg = ingest_mqtt()
    
       if (has_warning(mqtt_msg)):
            warnings = extract_warnings(mqtt_msg)
            for warn in warning:
                display(warn, RED, delay = 100)
        else:
            selected_data = extract_column(mqtt_msg, GREEN) 
"""
#serial = spi(device = 0, port = 0)

#device  = ssd1351(serial)
#with canvas(device) as draw:
#    draw.text((30, 40), "please work", fill = "white")
#sleep(1000)

if __name__ == "__main__":
    tests()
    main()
