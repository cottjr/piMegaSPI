/*************************************************************
 SPI_Hello_Raspi
   Configures an ATMEGA as an SPI slave and demonstrates
   bidirectional communication with an Raspberry Pi SPI master
   by repeatedly sending the text "Hello Raspi"
****************************************************************/

#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included


/***************************************************************
 Global Variables
  -hello[] is an array to hold the data to be transmitted
  -marker is used as a pointer in traversing data arrays
/***************************************************************/

unsigned char hello[] = {'H','e','l','l','o',' ',
                         'R','a','s','p','i','\n'};
byte marker = 0;
 

/***************************************************************  
 Setup SPI in slave mode (1) define MISO pin as output (2) set
 enable bit of the SPI configuration register 
****************************************************************/ 
                    
void setup (void)
{
  Serial.begin(250000);   // Serial:  0(RX), 1(TX) => use the highest possible rate to minimize drag on the CPU
                          // e.g. https://forum.arduino.cc/index.php?topic=76359.0
                          // e.g. https://www.quora.com/What-is-the-baud-rate-and-why-does-Arduino-have-a-baud-rate-of-9-600

    Serial.println("entering setup");

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

    Serial.println("completed setup");

}  

/***************************************************************  
 Loop until the SPI End of Transmission Flag (SPIF) is set
 indicating a byte has been received.  When a byte is
 received, load the next byte in the Hello[] array into SPDR
 to be transmitted to the Raspberry Pi, and increment the marker.
 If the end of the Hell0[] array has been reached, reset
 marker to 0.
****************************************************************/

void loop (void)
{

  if((SPSR & (1 << SPIF)) != 0)
  {
    // Serial.println(marker);
    SPDR = hello[marker];
    marker++;
   
    if(marker > sizeof(hello))
    {
      Serial.println("done");
      marker = 0;
    }  
  }
}