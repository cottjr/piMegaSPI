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
Declare Functions
***********************************************************/

int spiTxRx(unsigned char txDat);


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

   fd = open("/dev/spidev0.0", O_RDWR);

   unsigned int speed = 1000000;
   ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  printf ("made it past the register initialization...\n");
  cout << "does cout actually work?";

   while (1)
   {

      for (int i = 0; i < sizeof(hello); i++)
      {
         result = spiTxRx(hello[i]);
         cout << result;
         usleep (10);
      }

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
