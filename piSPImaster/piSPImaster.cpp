/**********************************************************
 piSPImaster
  Configures an Raspberry Pi as an SPI master
   to provide bidirectional communication with an Arduino Slave.

  This sample code establishes an simple SPI protocol,
    intended for intial use within a 'DPRG level' robot.

  This sample provides a single-file framework
    - to use as reference code, to be embedded or leveraged in a larger Raspberry Pi program
      => initially, for use in a differential drive robot that mashes together & extends 2 open source designs
          DonkeyCar             https://www.donkeycar.com/ 
          DPRG 2016 Club Robot  
            https://github.com/dprg/2016-Club-Robot-Code
            https://github.com/dprg/2016-Club-Robot-Mech
            https://github.com/dprg/2016-Club-Robot-Encoder-Tests
    - to use as verification test for the corresponding Arduino Slave code
   
Compile String:
// be sure to pick up the wiringPi library per this http://wiringpi.com/reference/
g++ -o piSPImaster piSPImaster.cpp -I/usr/local/include -L/usr/local/lib -lwiringPi

***********************************************************/

// This protocol builds on lessons extended from
//  http://robotics.hobbizine.com/raspiduino.html
//  and 
//  https://roboticsbackend.com/raspberry-pi-master-arduino-uno-slave-spi-communication-with-wiringpi/

// This implementation assumes a 3.3v Pi connected to a 5v Arduino Mega 2560 via a TXS0108E level shifter,
// and requires the Raspberry Pi to explicitly enable level shifter output pins
// For this, need to set GPIO 25 to an output, and set it high
// https://www.digikey.com/en/maker/blogs/2019/how-to-use-gpio-on-the-raspberry-pi-with-c

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>

using namespace std;


/**********************************************************
Declare Global Variables
***********************************************************/
int fdSPI;              // pointer to SPI port
// unsigned char result;

// placeholder values for this sample / test code
// for a real instance, these values would be bound to dependent code.
unsigned char receivedByte1, receivedByte2, receivedByte3;
long receivedLong1, receivedLong2, receivedLong3;


/**********************************************************
Declare Functions to allow forward references
***********************************************************/
int spiTxRx(unsigned char txDat);
int doSPItransfer(char command, signed char TurnVelocity, signed char Throttle, long param1, long param2, long param3 );


// Simple demonstration / test loop
// Exercises this protocol in a sandbox
// Allows working on the basic protocol and structure
int main (void)
{

  receivedByte1 = 0;
  receivedByte2 = 0;
  receivedByte3 = 0;
  receivedLong1 = 0;
  receivedLong2 = 0;
  receivedLong3 = 0;

  printf("Initializing Raspberry Pi SPI port.\n");

  wiringPiSetup();  // set up the Pi library to enable directly controlling GPIO pins
  pinMode(6, OUTPUT); // set GPIO.6, ie. BCM 25 ie. physical pin 22 as output
  digitalWrite(6, HIGH); // set GPIO.6, high, to enable outputs on the SPI level translater from Raspberry to Arduino Mega

  /**********************************************************
  Setup SPI
  Open file spidev0.0 (chip enable 0) for read/write access
  with the file descriptor "fdSPI"
  Configure transfer speed (1MkHz)
  ***********************************************************/
  fdSPI = open("/dev/spidev0.0", O_RDWR);

  unsigned int speed = 1000000;
  ioctl (fdSPI, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  printf ("Raspberry Pi SPI port initialization complete.\n");
  
  while (1)
  {
    int xferSuccess = 0;
    xferSuccess = doSPItransfer('a', 0, 50, 25, 300, 549);
    if (xferSuccess == 1)
    {
      cout << "Transfer successful." << endl;
      cout << "Byte1: " << (char) receivedByte1 << endl;
      cout << "Byte2: " << (int) (signed char) receivedByte2 << endl;
      cout << "Byte3: " << (int) (signed char) receivedByte3 << endl;      
      cout << "Long1: " << receivedLong1 << endl;
      cout << "Long2: " << receivedLong2 << endl;
      cout << "Long3: " << receivedLong3 << endl;
      cout << endl;
    } else
    {
      cout << "Transfer failed. Transfer request initiated but no acknowledge received." << endl;
      cout << endl;
    } 

    sleep(2);
  }
}


/**********************************************************
spiTxRx
 Transmits one byte via the SPI device, and returns one byte
 as the result.

 Establishes a data structure, spi_ioc_transfer as defined
 by spidev.h and loads the various members to pass the data
 and configuration parameters to the SPI device via IOCTL

 Local variables txDat and rxDat are defined and passed by
 reference.  
***********************************************************/
int spiTxRx(unsigned char txDat)
{
 
  unsigned char rxDat;

  struct spi_ioc_transfer spi;

  memset (&spi, 0, sizeof (spi));

  spi.tx_buf        = (unsigned long)&txDat;
  spi.rx_buf        = (unsigned long)&rxDat;
  spi.len           = 1;

  ioctl (fdSPI, SPI_IOC_MESSAGE(1), &spi);

  return rxDat;
}


// Purpose:  
//    send commands and data from Pi master to Arduino Mega slave
//    receive data from Arduino Mega slave
//    MVP - initial value is simply to transfer TurnVelocity and Throttle commands between Pi master and Arduino slave
//    Is designed to provide a simple protocol that enables transferring lots more information later, 
//      by invoking these communication functions but without having to revist the protocol...
// Inputs
//    command - placeholder byte. intended to provide flexibility, and allow passing a command, 
//        for example, requesting some specific values, or by defining how to interpret following values
//    TurnVelocity -> intended to provide a 'turn velocity' setpoint value from the Pi master to the Arduino slave
//    Throttle -> intended to provide a 'throttle' setpoint value from the Pi master to the Arduino slave
//    param1, param2, param3 -> placeholders  for future development to allow transferring more info from the Pi master to the Arduino slave
//    external variables intended to receive values from SPI slave
// Algorithm
//    protocol / transfer mechanics mostly build on
//      http://robotics.hobbizine.com/raspiduino.html
//        and 
//      https://roboticsbackend.com/raspberry-pi-master-arduino-uno-slave-spi-communication-with-wiringpi/
// Expected values returned via SPI
//    1st byte / ie. received in exchange for 'command'
//      -> placeholder byte for future development. No specific meaning defined.
//    2nd byte / received in exchange for 'TurnVelocity'
//      -> intended to allow the Arduino slave to inform the Pi master of current commanded values
//      -> e.g. for the case where robot is being manually driven around by a controller connected to Arduino,
//      -> and the Pi master needs to observe those manual values, e.g. as training data for DonkeyCar
//    3rd byte / received in exchange for 'Throttle' 
//      -> intended to allow the Arduino slave to inform the Pi master of current commanded values
//      -> e.g. for the case where robot is being manually driven around by a controller connected to Arduino,
//      -> and the Pi master needs to observe those manual values, e.g. as training data for DonkeyCar
//    last 12 bytes / received in exchange for 'param1', 'param2' and 'param3
//      -> placeholder bytes for future development. No specific meaning defined.
// Outputs
//    returns 1 for successful transfer
//    returns -1 if transfer unsuccessful, e.g. slave times out / does not acknowledge start of burst
//    updates external variables with received/ buffered values at end of successful transfer
//    note: only updates external variables if it appears that transfer was successful
int doSPItransfer(char command, signed char TurnVelocity, signed char Throttle, long param1, long param2, long param3 )
{

  unsigned char byteFromSPI;
  bool ack;

// use unions to simplify references to variables by individual bytes or as different types
  union byteUnion
  {
    char asChar;
    signed char asSignedChar;
    unsigned char asUnsignedChar;
  };

  union longUnion       
  {
    long  asLong;
    unsigned char  asByte [4];
  };

  union byteUnion toSPIBufferByte1, toSPIBufferByte2, toSPIBufferByte3;
  union byteUnion fromSPIBufferByte1, fromSPIBufferByte2, fromSPIBufferByte3;

  union longUnion toSPIBufferLong1, toSPIBufferLong2, toSPIBufferLong3 ;
  union longUnion fromSPIBufferLong1, fromSPIBufferLong2, fromSPIBufferLong3 ;

  toSPIBufferByte1.asChar = command;
  toSPIBufferByte2.asSignedChar = TurnVelocity;
  toSPIBufferByte3.asSignedChar = Throttle;

  toSPIBufferLong1.asLong = param1;
  toSPIBufferLong2.asLong = param2;
  toSPIBufferLong3.asLong = param3;

  fromSPIBufferByte1.asUnsignedChar = 0;
  fromSPIBufferByte2.asUnsignedChar = 0;
  fromSPIBufferByte3.asUnsignedChar = 0;
  fromSPIBufferLong1.asLong = 0;
  fromSPIBufferLong2.asLong = 0;
  fromSPIBufferLong3.asLong = 0;

  /**********************************************************
  Start handshake sequence: send a one byte start code
  ('s' for start) and loop until receive a one byte 
  acknowledgment code ('a').
  (Note that the loop also sends the command byte while 
  still in handshake sequence to avoid wasting a transmit
  cycle.)
  ***********************************************************/
  int wdCounter = 0;
  do
  {
    ack = false;

    // this is the first'header' byte in a burst handshake
    spiTxRx('s');     
    usleep (10);

    // this is the second 'header' byte in a burst handshake
    // send a dummy value to fetch an acknowledge byte to determine if the slave is present in a state to proceed
    byteFromSPI = spiTxRx( 0 );
    if (byteFromSPI == 'a')
    {
      ack = true;
    }
    usleep (10);

    if (wdCounter > 17)   
    {
      // a prior partial transfer of 15 payload + 2 header bytes should've cleared by now
      // this limits disrupting the slave to a handful of SPI interrupts during each approx 4.5 ms attempt to connect
      return 0;   // hence -> declare an error, SPI slave unresponsive and leave receivedByte1..3 and receivedLong1..3 unchanged
    }
    wdCounter++;
  }
  while (ack == false);

  // the remaining bytes are payload, and are numbered 1 thru 15.
  //    hence, this protocol nominally consists of 17 byte bursts, 2 header + 15 payload

  // send Byte1 (command) and fetch Byte1 (command) from slave
  fromSPIBufferByte1.asUnsignedChar = spiTxRx(toSPIBufferByte1.asUnsignedChar);
  usleep (10);

  // send Byte2 (TurnVelocity) and fetch Byte2 (TurnVelocity) from slave
  fromSPIBufferByte2.asUnsignedChar = spiTxRx(toSPIBufferByte2.asUnsignedChar);
  usleep (10);

  // send Byte3 (Throttle) and fetch Byte3 (Throttle) from slave
  fromSPIBufferByte3.asUnsignedChar = spiTxRx(toSPIBufferByte3.asUnsignedChar);
  usleep (10);

  int i=0; 
  // send bytes 4 thru 7 (param1) and fetch response     
  for (i = 0; i <= 3; i++) 
  {
    fromSPIBufferLong1.asByte[i] = spiTxRx(toSPIBufferLong1.asByte[i]);
    usleep (10);
  }   

  // send bytes 8 thru 11 (param2) and fetch response     
  for (i = 0; i <= 3; i++) 
  {
    fromSPIBufferLong2.asByte[i] = spiTxRx(toSPIBufferLong2.asByte[i]);
    usleep (10);
  }   

  // send bytes 12 thru 15 (param3) and fetch response     
  for (i = 0; i <= 3; i++) 
  {
    fromSPIBufferLong3.asByte[i] = spiTxRx(toSPIBufferLong3.asByte[i]);
    usleep (10);
  }   

  // transfer appears to be successful
  // => hence, assign all received values to external variable dependencies
  // avoid corruptions mixing data from different transfers - take care to ensure this copy process is not interrupted
  receivedByte1 = fromSPIBufferByte1.asUnsignedChar;
  receivedByte2 = fromSPIBufferByte2.asUnsignedChar;
  receivedByte3 = fromSPIBufferByte3.asUnsignedChar;

  receivedLong1 = fromSPIBufferLong1.asLong;
  receivedLong2 = fromSPIBufferLong2.asLong;
  receivedLong3 = fromSPIBufferLong3.asLong;

  return 1;   // declare successful transfer, as best as we can measure that without some clever payload checksum or hash...
}
