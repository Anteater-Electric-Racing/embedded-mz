from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1351
from time import sleep
import paho.mqtt.subscribe as subscribe
import json
import itertools


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
    print(has_warning({"motor_fault": True}), " should be True (has_fault)")
    print(has_warning({"motor_fault": False}), " should be False (has_fault)")
    warnings = [w for w in get_warnings({"motor_fault" : True, "dc_main_wire_over_vault_fault": False})]
    print(warnings, "should only have motor_fault")

def ingest_mqtt():
   payload = json.loads(SUBSCRIPTION.payload)
   return payload

def warning_criteria(entry):
    #will be more sophisticated later to allow for multi level warnings
    return "fault" in entry[0] and entry[1] == True

def has_warning(payload : dict):
    for item in payload.items():
        if warning_criteria(item):
            return True
    return False

def get_warnings(payload : dict):
    yield from map(lambda x : x[0], filter(warning_criteria, payload.items()))

def main():
    parse_args()
    

"""
    while True: 
        mqtt_msg = ingest_mqtt()
    
       if (has_warning(mqtt_msg)):
            banner_display(get_warnings(mqtt_msg), RED, delay = 100)
        else:
            selected_data = extract_column(mqtt_msgi, SELECTED) 
            display(SELECTED, selected_data, GREEN)
"""
#serial = spi(device = 0, port = 0)

#device  = ssd1351(serial)
#with canvas(device) as draw:
#    draw.text((30, 40), "please work", fill = "white")
#sleep(1000)

if __name__ == "__main__":
    tests()
    main()
