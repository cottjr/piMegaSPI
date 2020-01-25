# PiSPImaster.py

#  https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all

import sys
import time
import spidev   # https://github.com/doceme/py-spidev

# import RPi.GPIO as GPIO     # https://learn.sparkfun.com/tutorials/raspberry-gpio/python-rpigpio-api
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0 #1

# Enable SPI
spi = spidev.SpiDev()

GPIO.setmode(GPIO.BOARD)    # https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/                           
GPIO.setup(22, GPIO.OUT)    # set GPIO.6, ie. BCM 25 ie. physical pin 22 as output
GPIO.output(22, GPIO.HIGH)  # set GPIO.6, high, to enable outputs on the SPI level translater from Raspberry to Arduino Mega

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 1000000
spi.mode = 0

print ('Raspberry Pi SPI master port initialization complete.\n')

# int doSPItransfer(char command, signed char TurnVelocity, signed char ForwardThrottle, signed char SidewaysThrottle, long param1, long param2, long param3 )
def doSPItransfer(): # command, TurnVelocity, ForwardThrottle, SidewaysThrottle, param1, param2, param3 ):
    print ('Now starting doSPItransfer.\n')
    wdCounter = 0
    ack = False
    while not ack:
        # exchange a first'header' byte in a burst handshake, and throw away the recieved byte
        msg = [ord("s")]
        # note: piSPImaster.cpp spiTxRx() => maps to this python spidev library spi.xfer()
        byteFromSPI = spi.xfer(msg)
        time.sleep(0.00001)

        # exchange a second 'header' byte in a burst handshake
        # send a dummy value to fetch an acknowledge byte to determine if the slave is present in a state to proceed
        msg = [0]
        byteFromSPI = spi.xfer(msg)
        print('byeFromSPI:',byteFromSPI[0])
        if byteFromSPI[0] == ord('a'):
            print ('Aha- received an "a".\n')
            ack = True
        time.sleep(0.00001)

        if wdCounter > 17:
            print ('hit the 17 byte attempt, then moving on.\n')
            #   a prior partial transfer of 15 payload + 2 header bytes should've cleared by now
            #   this limits disrupting the slave to a handful of SPI interrupts during each approx 4.5 ms attempt to connect
            return 0    # // hence -> declare an error, SPI slave unresponsive and leave receivedByte1..3 and receivedLong1..3 unchanged
        wdCounter += 1

doSPItransfer()


i = 0
while True:
    i += 1

    if i == 1 or i == 2:
        print ('and now hit the 1st or 2nd iteration of while loop after that....\n')
    msg = [ord("s")]
    # note: piSPImaster.cpp spiTxRx() => maps to this python spidev library spi.xfer()
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



