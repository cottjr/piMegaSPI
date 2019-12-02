/**********************************************************
 piSPImaster
   Configures an Raspberry Pi as an SPI master and  
   demonstrates bidirectional communication with an 
   Arduino Slave by repeatedly sending the text
   "Hello Arduino" and receiving a response
   
Compile String:
// doesn't work with wiring library references
g++ -o piSPImaster piSPImaster.cpp

// need to pick up the wiringPi library per this http://wiringpi.com/reference/
g++ -o piSPImaster piSPImaster.cpp -I/usr/local/include -L/usr/local/lib -lwiringPi

***********************************************************/

// sample follows http://robotics.hobbizine.com/raspiduino.html

// for Carl's implementation, with a TXS0108E level shifter
// need the Raspberry Pi to explicitly enable level shifter output pins
// for this, need to set GPIO 25 to an output, and set it high
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
unsigned char hello[] = {'H','e','l','l','o',' ',
                           'A','r','d','u','i','n','o'};
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


/**********************************************************
Main
  Setup SPI
    Open file spidev0.0 (chip enable 0) for read/write 
      access with the file descriptor "fd"
    Configure transfer speed (1MkHz)
  Start an endless loop that repeatedly sends the characters
    in the hello[] array to the Ardiuno and displays
    the returned bytes
***********************************************************/

int main (void)
{

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
An endless loop that repeatedly sends the demonstration
commands to the Arduino and displays the results
***********************************************************/
   while (1)
   {

// // version 1 simply sent a 'hello' string to the ardunio
// //      for (int i = 0; i < sizeof(hello); i++)
// //      {
// //         result = spiTxRx(hello[i]);
// //         cout << result;
// //         usleep (10);
// //      }

// version 2 sends a single byte command and two two-byte parameters
      results = sendCommand('a', 510, 655);

      cout << "Addition results:" << endl;
      cout << "510 + 655 = " <<  (int)(results) << endl;


      results = sendCommand('s', 1000, 250);

      cout << "Subtraction results:" << endl;
      cout << "1000 - 250 = " <<  (int)(results) << endl <<endl; 

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
