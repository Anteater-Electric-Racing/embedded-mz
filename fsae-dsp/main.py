from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1351
from luma.core.virtual import viewport
from time import sleep
import paho.mqtt.subscribe as subscribe
from PIL import ImageFont
from pathlib import Path
import json
import itertools
import functools


#Ingest data
#Process
#   Extract tag
#   Extract_warning
#Display

RED = "blue" # red is blue and blue is red
GREEN = "green"
selected = "pack_current"
topic = "telemetry"
host = "127.0.0.1"
subscription = ""
serial = spi(device = 0, port = 0)

device  = ssd1351(serial)

def make_font(name, size):
    font_path = str(Path(__file__).resolve().parent.joinpath('fonts', name))
    return ImageFont.truetype(font_path, size)

WARNING_FONT = make_font("C&C Red Alert [INET].ttf", 24)
SELECTED_FONT = make_font("C&C Red Alert [INET].ttf", 10)
VALUE_FONT = make_font("C&C Red Alert [INET].ttf", 24)

def parse_args():
    global subscription
    #planned flags:
    # -h help
    # --select what column
    # --warn [none/warn/fault] look for faults or faults + warns or ignore faults and wanrs 
    print("parse_args stub")
    subscription = subscribe.simple(topic, hostname=host)
    #this function will also have to reintialize the subscriber but anyways

def tests():
    #very scuffed
    parse_args()
    print(mqtt_msg := ingest_mqtt())
    print(type(mqtt_msg), " should be dict")
    print(has_warning({"motor_fault": True}), " should be True (has_fault)")
    print(has_warning({"motor_fault": False}), " should be False (has_fault)")
    ingress = {"motor_fault" : True, "dc_main_wire_over_vault_fault": False, "motor_stall_fault": True, "motor_speed":10.0} 
    warnings = [w for w in get_warnings(ingress)]
    print(warnings, " should only have motor_fault, and motor_stall_fault")
    print(generate_warnings_string(warnings), " should be motor_fault motor_stall_fault")
    print("Displaying warnings")
    scroll_display(warnings, font = WARNING_FONT, speed = 2)
    print(extract_column(ingress, "motor_speed"), " should be 10.0")
    print("Displaying info")
    i = 10.0
    while i < 100:
        display("motor_speed", i)
        i+=1

def ingest_mqtt():
   payload = json.loads(subscription.payload)
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

#if this is spelled wrong, mb, I don't have autocorrect on
def concatenate_strings(a : str, b : str):
    return a + "  %%  " + b

def generate_warnings_string(warnings : list[str]):
    return functools.reduce(concatenate_strings, warnings)

def scroll_display(warnings : list[str], speed= 1, fill = RED, font = None):
    message = generate_warnings_string(warnings)
    x = device.width
    with canvas(device) as draw:
        left, top, right, bottom = draw.textbbox((x,0), message, font = font)
        w, h = right - left, bottom -top 

    virtual = viewport(device, width=w + x + x, height = max(h, device.height))
    with canvas(virtual) as draw:
        draw.text((x,device.height/2 - h/2), message, fill = fill, font = font)

    i = 0
    while i < w + x:
        virtual.set_position((i,0))
        i += speed
        sleep(0.025)

def extract_column (msg, sel):
    return msg[sel]

def display(sel, data, fill = GREEN):
    data = str(data)
    with canvas(device) as draw:
        left, top, right, bottom = draw.textbbox((0,0), data, font = VALUE_FONT)
        w, h = right - left, bottom -top 

        draw.text((0,0), sel, font = SELECTED_FONT, fill = fill)
        draw.text((device.width/2 - w/2, device.height/2 - h/2), data, font = VALUE_FONT, fill = fill)
    sleep(0.025)

def main():
    parse_args()
    

"""
    while True: 
        mqtt_msg = ingest_mqtt()
    
       if (has_warning(mqtt_msg)):
            scroll_display(get_warnings(mqtt_msg), RED)
        else:
selected_data = extract_column(mqtt_msgi, selected) 
            display(selected, selected_data, GREEN)
"""
#serial = spi(device = 0, port = 0)

#device  = ssd1351(serial)
#with canvas(device) as draw:
#    draw.text((30, 40), "please work", fill = "white")
#sleep(1000)

if __name__ == "__main__":
    tests()
    main()
