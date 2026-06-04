from luma.core.interface.serial import spi
from luma.core.render impotr canvas
from luma.oled.device import ssd1351
from time import sleep

serial = spi()

device  = ssd1351(serial)

with canvas(device) as draw:
    draw.text((30, 40), "please work", fill = "white")
    sleep(1000)
