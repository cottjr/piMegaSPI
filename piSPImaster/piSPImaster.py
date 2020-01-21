# PiSPImaster.py

#  https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all

import sys
import time
import spidev   # https://github.com/doceme/py-spidev

# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0 #1

# Enable SPI
spi = spidev.SpiDev()

# ToDo! =>>>>>>  port this C to Python to enable the level translator
#   wiringPiSetup();  // set up the Pi library to enable directly controlling GPIO pins
#   pinMode(6, OUTPUT); // set GPIO.6, ie. BCM 25 ie. physical pin 22 as output
#   digitalWrite(6, HIGH); // set GPIO.6, high, to enable outputs on the SPI level translater from Raspberry to Arduino Mega


# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 1000000
spi.mode = 0

print ('Raspberry Pi SPI master port initialization complete.\n')
print ('--> EXCEPT - still need to enable GPIO.6 w/in this code, to enable the level translator.\n\n')







# note: piSPImaster.cpp spiTxRx() => maps to this python spidev library spi.xfer()


i = 1
while True:

    msg = [ord("s")]
    result = spi.xfer(msg)

    time.sleep(0.00001)

    msg = [0]
    result = spi.xfer(msg)

    time.sleep(0.00001)

    msg = [ord("R")]
    result = spi.xfer(msg)

    time.sleep(0.00001)

    # The third character
    msg = [0x7d]
    # msg.append(i)
    result = spi.xfer(msg)

    time.sleep(0.00001)

    # The last character
    msg = [0x7e]
    # msg.append(i)
    result = spi.xfer(msg)

    time.sleep(0.00001)



    # Pause so we can see them
    time.sleep(0.1)



