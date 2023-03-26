import os
from machine import Pin, SDCard


# *** SD card *************************
sd = SDCard(slot=2, width=1, sck=Pin(18), mosi=Pin(23), miso=Pin(19),cs=Pin(5))
os.mount(sd, '/sd')
os.listdir()
fs = open('hello.txt', 'w')
fs.write('print("hello!")')
#while True:
#    pass
# ------- debug stop -------
