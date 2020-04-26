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

# Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0  # 1

# Enable SPI
spi = spidev.SpiDev()

# https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
GPIO.setmode(GPIO.BOARD)
# set GPIO.6, ie. BCM 25 ie. physical pin 22 as output
GPIO.setup(22, GPIO.OUT)
# set GPIO.6, high, to enable outputs on the SPI level translater from Raspberry to Arduino Mega
GPIO.output(22, GPIO.HIGH)

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 1000000
spi.mode = 0

print('Raspberry Pi SPI master port initialization complete.\n')

# keep track of transfer error count detected by Pi SPI Master
errorCountSPIrx = 0

# https://wiki.python.org/moin/BitManipulation
# counts the number of bits needed to represent an integer


def bitLen(int_type):
    length = 0
    while (int_type):
        int_type >>= 1
        length += 1
    return(length)


# --------------------ToDo----Refactor Next Few Functions----------------------------------------------------------------------
# Need to refactor the next 3 fuctions
# Consider the struct module apparently available and advancing since early versions of Python
#  https://docs.python.org/3/library/struct.html
#  https://tutorialspoint.dev/language/python/struct-module-python
#  https://pymotw.com/2/struct/

# Convert any Python signed integer to a clamped range of -128..+127 and then to a 2's complement range of 0..255
# matches range to C signed char which ranges from -128 to +127
# Algorithm
#  use abstract math versus bit twiddling / ie. rely on implicit byte interpretation by SPI receiver of range of 0..255
#  ie. as Signed integer        -128    -1      0   1   127
#  ie. as 2's comp Signed Char  +128    255     0   1   127
# clamp value to allowed range in case of function misuse
def signedIntegerToCSignedChar(int_type):
    if int_type <= -128:
        return 128
    elif int_type < 0:
        return (256 + int_type)
    elif int_type < 127:
        return int_type
    else:
        return 127

# Convert any Python integer representing a C signed char (range of 0..255 representing a 2's complement number) to a Python signed integer in range of -128..+127
# Algorithm
#  use abstract math versus bit twiddling / ie. rely on implicit byte interpretation by SPI transmitter of range of 0..255
#  ie. as Signed integer        -128    -1      0   1   127
#  ie. as 2's comp Signed Char  +128    255     0   1   127
# clamp value to zero in case of function misuse


def CSignedCharToSignedInteger(int_type):
    if (int_type < 0) or (int_type > 255):
        return 0
    elif int_type <= 127:
        return int_type
    else:
        return (int_type - 256)

# Convert any Python integer representing a 4 byte C Long (range of 0 .. +4,294,967,295 representing a 2's complement number) to a Python signed integer in range of -2,147,483,648 .. +2,147,483,647
# Algorithm
#  use abstract math versus bit twiddling / ie. rely on implicit byte interpretation by SPI transmitter of range of 0..255
# ie. as signed integer     -2,147,483,648  -1              0   1   2,147,483,647
# ie. as 2's comp Long      +2,147,483,648  +4,294,967,295  0   1   2,147,483,647
# clamp value to zero in case of function misuse


def CLongToSignedInteger(long_type):
    if (int_type < 0) or (int_type > 4294967295):
        return 0
    if long_type <= 2147483647:
        return long_type
    else:
        return (long_type - 4294967296)

# --------------------ToDo----Refactor Previous Few Functions----------------------------------------------------------------------


# cheesy quick-n-dirty, use a global to store loopback values between iterations
# Note: this means that operator command values received from the SPI slave are not fed back to the SPI slave until the next transfer cycle
#   hence this can add considerable lag to control loops, e.g. if response to operator commands has to wait for a round trip from slave -> master -> slave
#   this effect is considerable when the SPI iteration cycle is set to 0.9 sec, or even 0.1 sec
#   In fact, even 0.05 sec gives unacceptable lag with ClubCarDonkyBot circa (commit 062acc3) plus changes towards the next commit (ie. 1st commit in branch donkeyCarIntegration)
payloadToSPIBuffer = bytearray(16)

print()
print('-------------------------------')
# int doSPItransfer(char command, signed char TurnVelocity, signed char ForwardThrottle, signed char SidewaysThrottle, long param1, long param2, long param3 )


def doSPItransfer(command, TurnVelocity, ForwardThrottle, SidewaysThrottle, param1, param2, param3, errorCountSPIrx):
    print('\nNow starting doSPItransfer.')

    payloadFromSPIBuffer = bytearray(16)

    # For this loopBack branch, ignore parameters passed in, and store the values received from SPI slave on this iteration,
    # to be sent back to SPI slave on next interation
    # payloadToSPIBuffer[0] = command
    # payloadToSPIBuffer[1] = signedIntegerToCSignedChar(TurnVelocity)
    # payloadToSPIBuffer[2] = signedIntegerToCSignedChar(ForwardThrottle)
    # payloadToSPIBuffer[3] = signedIntegerToCSignedChar(SidewaysThrottle)
    # ToDo- refactor/complete CLongToSignedInteger() exercise to extract general data passed in param1, param2 and param3

    wdCounter = 0
    ack = False
    while not ack:
        # exchange a first'header' byte in a burst handshake, and throw away the recieved byte
        # note: piSPImaster.cpp spiTxRx() => maps to this python spidev library spi.xfer()
        spi.xfer([ord("s")])
        time.sleep(0.00007)

        # exchange a second 'header' byte in a burst handshake
        # send a dummy value to fetch an acknowledge byte to determine if the slave is present in a state to proceed
        byteListFromSPI = spi.xfer([0])
        # print('type of byteFromSPI ', type(byteFromSPI),'type of byteFromSPI[0] ', type(byteFromSPI[0]), 'byteFromSPI:',byteFromSPI[0], ' , in hex ', hex(byteFromSPI[0]), ' , bit length ', bitLen(byteFromSPI[0]))
        # print(byteFromSPI[0].to_bytes(1, byteorder="little", signed=True))
        if byteListFromSPI[0] == ord('a'):
            ack = True
        time.sleep(0.00007)

        if wdCounter > 17:
            print(
                '-- transfer fail. Hit the 17 byte timeout. No initial handshake acknowledgement.')
            #   a prior partial transfer of 15 payload + 2 header bytes should've cleared by now
            #   this limits disrupting the slave to a handful of SPI interrupts during each approx 4.5 ms attempt to connect
            errorCountSPIrx += 1    # bump the SPI transfer error count
            print('-- SPI transfer error count: ', errorCountSPIrx)
            return 0    # // hence -> declare an error, SPI slave unresponsive and leave receivedByte1..3 and receivedLong1..3 unchanged
        wdCounter += 1

#   the remaining bytes are payload, and are numbered 0 thru 15.
#   hence, this protocol nominally consists of 19 byte bursts, 2 header + 16 payload + 1 handshaking
    for x in range(16):
        payloadFromSPIBuffer[x] = spi.xfer([payloadToSPIBuffer[x]])[0]
        time.sleep(0.00007)

#   this is the last 'handshaking' byte in a burst
#   it is intended for master and slave to indicate to each other that they expect this to be the final byte transferred in a burst
#   => and give confidence (albeit not certainty) that the payload bytes transferred between 'a' and 'z' can be trusted
    # note - Pi master sends a lower case 'z'
    byteListFromSPI = spi.xfer([ord("z")])
    # semi-ensure a minimum time between attempts. Simple approach for now. A better / future approach would use semaphore.
    time.sleep(0.00007)

    # note - Arduino slave must send an upper case 'Z'
    if byteListFromSPI[0] == ord('Z'):
        # transfer appears to be successful
        # => hence, assign all received values to external variable dependencies
        # avoid corruptions mixing data from different transfers - take care to ensure this copy process is not interrupted
        print('byteListFromSPI (final acknowledge): ',
              byteListFromSPI, chr(byteListFromSPI[0]))
        print('payloadFromSPIBuffer: ', payloadFromSPIBuffer)
        print('command: ', payloadFromSPIBuffer[0], chr(
            payloadFromSPIBuffer[0]))
        print('TurnVelocity: ', CSignedCharToSignedInteger(
            payloadFromSPIBuffer[1]))
        print('ForwardThrottle: ', CSignedCharToSignedInteger(
            payloadFromSPIBuffer[2]))
        print('SidewaysThrottle: ', CSignedCharToSignedInteger(
            payloadFromSPIBuffer[3]))
        # ToDo- print remaining payload values
        print('SPI transfer error count: ', errorCountSPIrx)

        # since received values assumed good, save them to loop back for the next iteration
        for x in range(16):
            payloadToSPIBuffer[x] = payloadFromSPIBuffer[x]

        # declare successful transfer, as best as we can measure that without some clever payload checksum or hash...
        return 1

    errorCountSPIrx += 1    # bump the SPI transfer error count
    print('-- transfer fail. Initial handshake succeeded, but final acknowledgment failed.')
    print('-- SPI transfer error count: ', errorCountSPIrx)
    return 2  # -> declare an error, transfer burst started as expected, but then SPI slave appears to have gotten out of sync during the burst



# ToDo -> I think we want to make a bytearray, assign each received byte to it during the transfer, then manipulate it after the transfer is complete
# ie. something like the following, except, the SPI transfer returns signed integers
#  something from here https://www.delftstack.com/howto/python/how-to-convert-int-to-bytes-in-python-2-and-python-3/
#   e.g. (-127).to_bytes(1, byteorder="little", signed=True)
# NO WAIT -> perhaps the integer will come across as 0..255, and we don't need to do the conversion until later, when interpreting the byte!
#  -> perhaps we need to ship known bytes from the Arduino Mega across SPI, to verify this...  Hmmm...
# print('-------------------------------')
# print('bytearray experiments')
# myByteArray = bytearray(3)
# myByteArray[0] = 254
# myByteArray[1] = 17
# myByteArray[2] = 179
# # myByteArray[2] = -4 # -> must be in range 0..255
# print(type ((-127).to_bytes(1, byteorder="little", signed=True)) )
# print( (-127).to_bytes(1, byteorder="little", signed=True) )
# # myByteArray[2] = (-127).to_bytes(1, byteorder="little", signed=True)
# print(myByteArray)
# print()
# print('-------------------------------')
# a = b'0x21'
# print('a is', a, ' type ', type(a) ) #, ' left shifted 8 bits is ', a << 8 )
# print()
print('-------------------------------')
i = 0
while True:
    i += 1

    if i == 1:  # or i == 2:
        print(
            'reset the mega error counters on the 1st or 2nd iteration of while loop....\n')
        print('SPI exchange result: ', doSPItransfer(
            ord('R'), 0, 0, 0, 0, 0, 0, errorCountSPIrx))

    print('SPI exchange result: ', doSPItransfer(
        103, -125, -1, 37, 356, -94287, 5824498, errorCountSPIrx))

    # Vary the SPI exchange rate by changing the delay between SPI transfer cycles
    time.sleep(.02)

