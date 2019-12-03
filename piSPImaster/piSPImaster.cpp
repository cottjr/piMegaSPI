/**********************************************************
 piSPImaster
  Configures an Raspberry Pi as an SPI master and  
   demonstrates bidirectional communication with an Arduino Slave.
  
  
  Raspberry Pi repeatedly sends alternating math problems for the Arduino.
  The Arduino is expected to accept a math operand (add or subtract) and two parameters,
  and then calculate the result, and provide the result to the Raspberry Pi 'in the same burst'
  which provided the operand and parameters.

  This sample code is used to explore SPI interactions and verify a simple protocol, capable
  of transmitting multiple bytes and data types.
   
Compile String:
// be sure to pick up the wiringPi library per this http://wiringpi.com/reference/
g++ -o piSPImaster piSPImaster.cpp -I/usr/local/include -L/usr/local/lib -lwiringPi

***********************************************************/

// Prior initial / polled sample follows http://robotics.hobbizine.com/raspiduino.html
// This interrupt sample builds on that following https://roboticsbackend.com/raspberry-pi-master-arduino-uno-slave-spi-communication-with-wiringpi/

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
int fd;
unsigned char result;


/**********************************************************
Housekeeping variables
***********************************************************/
int results;


/**********************************************************
Declare Functions
***********************************************************/
int spiTxRx(unsigned char txDat);
int sendCommand(char i, int j, int k);



int main (void)
{

unsigned char dummyByte;

  printf("OK, lets get this party started\n");

  wiringPiSetup();  // set up the Pi library to enable directly controlling GPIO pins
  pinMode(6, OUTPUT); // set GPIO.6, ie. BCM 25 ie. physical pin 22 as output
  digitalWrite(6, HIGH); // set GPIO.6, high, to enable outputs on the SPI level translater from Raspberry to Arduino Mega


  /**********************************************************
  Setup SPI
  Open file spidev0.0 (chip enable 0) for read/write access
  with the file descriptor "fd"
  Configure transfer speed (1MkHz)
  ***********************************************************/
   fd = open("/dev/spidev0.0", O_RDWR);

   unsigned int speed = 1000000;
   ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  printf ("made it past the register initialization...\n");
  cout << "does cout actually work?";

  /**********************************************************
  An endless loop that repeatedly sends simple math problems
  to the Arduino, captures the results & writes the results to console
  ***********************************************************/
   while (1)
   {

  // this version sends a single byte operand command and two separate two-byte parameters
  // Each execution of sendCommand() transfers 8 bytes in each direction
      results = sendCommand('a', 510, 655);

      cout << "Addition results:" << endl;
      cout << "510 + 655 = " <<  (int)(results) << endl;

  //robustness test
  dummyByte = spiTxRx(5);  // send a spurious byte to verify that the Arduino rejects it.
  usleep (10);  

      results = sendCommand('s', 1000, 250);

      cout << "Subtraction results:" << endl;
      cout << "1000 - 250 = " <<  (int)(results) << endl <<endl; 

  //robustness test
  usleep (200); 
  dummyByte = spiTxRx(5);  // send a spurious byte to verify that the Arduino rejects it.
  usleep (10); 

      sleep(1);
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

  ioctl (fd, SPI_IOC_MESSAGE(1), &spi);

  return rxDat;
}


/**********************************************************
sendCommand
 Demonstration of a protocol that uses the spiTxRx function
 to send a formatted command sequence/packet to the Arduino
 one byte at and capture the results
***********************************************************/
int sendCommand(char command, int j, int k)
{

unsigned char resultByte;
bool ack;

/**********************************************************
Unions allow variables to occupy the same memory space
a convenient way to move back and forth between 8-bit and
16-bit values etc.

Here three unions are declared: two for parameters to be 
passed in commands to the Arduino and one to receive
the results
***********************************************************/

union p1Buffer_T       
{
  int p1Int;
  unsigned char  p1Char [2];
} p1Buffer;

union p2Buffer_T      
{
  int p2Int;
  unsigned char  p2Char [2];
} p2Buffer;

union resultBuffer_T     
{
  int resultInt;
  unsigned char  resultChar [2];
} resultBuffer;


  p1Buffer.p1Int = j;
  p2Buffer.p2Int = k;
  resultBuffer.resultInt = 0;

  /**********************************************************
  An initial handshake sequence sends a one byte start code
  ('c') and loops endlessly until it receives the one byte 
  acknowledgment code ('a') and sets the ack flag to true.
  (Note that the loop also sends the command byte while 
  still in handshake sequence to avoid wasting a transmit
  cycle.)
  ***********************************************************/
  do
  {
    ack = false;

    spiTxRx('c');
    usleep (10);


    resultByte = spiTxRx(command);
    if (resultByte == 'a')
    {
      ack = true;
    }
    usleep (10);  

   }
  while (ack == false);

  /**********************************************************
  Send the parameters one byte at a time.
  ***********************************************************/

  spiTxRx(p1Buffer.p1Char[0]);
  usleep (10);


  spiTxRx(p1Buffer.p1Char[1]);
  usleep (10);


  spiTxRx(p2Buffer.p2Char[0]);
  usleep (10);

  spiTxRx(p2Buffer.p2Char[1]);
  usleep (10);

  /**********************************************************
  Push two more zeros through so the Arduino can return the
  results
  ***********************************************************/

  resultByte = spiTxRx(0);
  resultBuffer.resultChar[0] = resultByte;
  usleep (10);


  resultByte = spiTxRx(0);
  resultBuffer.resultChar[1] = resultByte;
  return resultBuffer.resultInt;

}
