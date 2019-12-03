/*************************************************************
 megaSPIslave
  Configures an Arduino as an SPI slave, and demonstrates bidirectional
  communication with a Raspberry Pi conmfigured as an SPI master.
  In this implementation, the Arduino uses a polling mechanism to trigger actions when a new byte arrives via SPI.
    
  Raspberry Pi repeatedly sends alternating math problems for the Arduino.
  The Arduino is expected to accept a math operand (add or subtract) and two parameters,
  and then calculate the result, and provide the result to the Raspberry Pi 'in the same burst'
  which provided the operand and parameters.

  This sample code is used to explore SPI interactions and verify a simple protocol, capable
  of transmitting multiple bytes and data types.
****************************************************************/

#include <arduino.h>
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included
#include <SPI.h>

/***************************************************************
 Global Variables
  -receiveBuffer[] and dat are used to capture incoming data
   from the Raspberry Pi
  -marker is used as a pointer to keep track of the current
   position in the incoming data packet
  -marker is used as a pointer in traversing data arrays
/***************************************************************/

unsigned char receiveBuffer[5];
unsigned char dat;                         
byte marker = 0;
 
#define digTP26 26  // digital test point #1 (pin 26) // CPU status monitoring via o'scope
#define digTP27 27  // digital test point #2 (pin 27) // CPU status monitoring via o'scope

/*************************************************************
 Unions allow variables to occupy the same memory space a 
 convenient way to move back and forth between 8-bit and 
 16-bit values etc.  Here three unions are declared: 
 two for parameters that are passed in commands to the Arduino 
 and one to receive  the results 
 ***************************************************************/

union       
  {
  int p1Int;
  unsigned char  p1Char [2];
  } p1Buffer;

union       
  {
  int p2Int;
  unsigned char p2Char [2];
  } p2Buffer;


union       
  {
  int resultInt;
  unsigned char  resultChar [2];
  } resultBuffer;


#define digTP26 26  // digital test point #1 (pin 26) // CPU status monitoring via o'scope
#define digTP27 27  // digital test point #2 (pin 27) // CPU status monitoring via o'scope


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

    // Digital Test Points
  pinMode (digTP26, OUTPUT);
  pinMode (digTP27, OUTPUT);
  digitalWrite(digTP26, LOW);
  digitalWrite(digTP27, LOW);

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  Serial.println("completed setup");

}  

/***************************************************************  
 This version Loops until the SPI End of Transmission Flag (SPIF) is set,
 which indicates that a byte has been received, and triggers a call to spiHandler().
****************************************************************/
void loop (void)
{

  if((SPSR & (1 << SPIF)) != 0)
  {
    digitalWrite(digTP26, HIGH);
    spiHandler();
    digitalWrite(digTP26, LOW);    
  }
}

// Purpose
//  Assess arrival of another byte via SPI
// Algorithm
//  Ignore it, Reset the transfer protocol state machine, or add the latest transferred byte to an 'being catured' buffer
/***************************************************************  
 spiHandler
   Uses the marker variable to keep track current position in the
   incoming data packet and execute accordingly
   0   - wait for to receive start byte - once received send
         the acknowledge byte
   1   - the command to add or subtract
   2-5 - two integer parameters to be added or subtracted
       - when the last byte (5) is received, call the
         executeCommand function and load the first byte of the
         result into SPDR
   6   - transmit the first byte of the result and load the 
         second byte into SPDR
   7   - transmit the second byte of of the result and reset
         the marker   
****************************************************************/
void spiHandler()
{
  switch (marker)
  {
  case 0:
    dat = SPDR;
    if (dat == 'c')
    {
      SPDR = 'a';
      marker++;
    } 
    break;    
  case 1:
    receiveBuffer[marker-1] = SPDR;
    marker++;
    break;
  case 2:
    receiveBuffer[marker-1] = SPDR;
    marker++;
    break;
  case 3:
    receiveBuffer[marker-1] = SPDR;
    marker++;
    break;
  case 4:
    receiveBuffer[marker-1] = SPDR;
    marker++;
    break;
  case 5:
    receiveBuffer[marker-1] = SPDR;
    marker++;
    executeCommand();
    SPDR = resultBuffer.resultChar[0];    
    break;    
  case 6:
    marker++;
    SPDR = resultBuffer.resultChar[1]; 
    break;   
  case 7:
    dat = SPDR;
    marker=0;
  }

}

/***************************************************************  
 executeCommand
   When the complete 5 byte command sequence has been received
   reconstitute the byte variables from the receiveBuffer
   into integers, parse the command (add or subtract) and perform
   the indicated operation - the result will be in resultBuffer
****************************************************************/
void executeCommand(void)
{

 p1Buffer.p1Char[0]=receiveBuffer[1];
 p1Buffer.p1Char[1]=receiveBuffer[2];
 p2Buffer.p2Char[0]=receiveBuffer[3];
 p2Buffer.p2Char[1]=receiveBuffer[4];
 
 // Note: avoid the temptation to place Serial.println() in this function
 // => because executeCommand() is called w/in an ISR,
 //    including Serial.println() here would break behavior...
 if(receiveBuffer[0] == 'a')
 {
   resultBuffer.resultInt = p1Buffer.p1Int + p2Buffer.p2Int;
  }
 else if (receiveBuffer[0] == 's')
 {
  resultBuffer.resultInt = p1Buffer.p1Int - p2Buffer.p2Int;
 }
} 