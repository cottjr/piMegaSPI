/*************************************************************
 megaSPIslave
  Configures an Arduino as an SPI slave
   to provide bidirectional communication with a Raspberry Pi master.

  This sample code establishes an simple SPI protocol,
    intended for intial use within a 'DPRG level' robot.

  This sample provides a single-file framework
    - to use as reference code, to be embedded or leveraged in a larger Arduino program
      => initially, for use in a differential drive robot that mashes together & extends 2 open source designs
          DonkeyCar             https://www.donkeycar.com/ 
          DPRG 2016 Club Robot  
            https://github.com/dprg/2016-Club-Robot-Code
            https://github.com/dprg/2016-Club-Robot-Mech
            https://github.com/dprg/2016-Club-Robot-Encoder-Tests  
****************************************************************/

#include <arduino.h>
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included
#include <SPI.h>

// SPI service state machine variables
volatile unsigned char receiveBuffer[15]; // temporary buffer for bytes coming from the SPI master
volatile unsigned char sendBuffer[2][15]; // temporary buffer for bytes to be sent to SPI master in the next burst
                                          // double buffer allows setting next burst values even if a current burst is in progress
volatile unsigned char sendBufferSelect = 0;  // currently seleted sendBuffer. start with the 0th sendBuffer                                                                
volatile unsigned char toggleSendBuffer = 0;  // flag to indicate whether to use the other sendBuffer on the next SPI transfer

volatile signed char SPIxferIndex = -2;   // state machine tracker- index to byte within incoming SPI burst
                                    // start at -2 to account for 2 header bytes in this protocol

volatile unsigned long SPIwdPriorMillis;  // watchdog timer for SPI state machine

volatile unsigned char SPIxferInProgress = 0; // flag using functions to determine whether the receiveBuffer can be trusted, ie. with WIP or a complete set of bytes from a single transfer

volatile unsigned char newSPIdataAvailable = 0; // flag for using functions to determine when new data has been received

volatile unsigned int errorCountSPIrx = 0;  // let's track apparent xfer failures

// test points to monitor CPU status via o'scope
#define digTP28 28  // digital test point #3 (Arduino Mega pin 28) 
#define digTP29 29  // digital test point #4 (Arduino Mega pin 29) 

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

// placeholder variables to provide interface between the SPI service and the functions using the SPI service
// ToDo -> eventually refactor these for more clean & abstracted interface to SPI service
signed char TurnVelocityFromMega = 0;
signed char ThrottleFromMega = 0;

char commandFromPi = 0;
signed char TurnVelocityFromPi = 0;
signed char ThrottleFromPi = 0;
long param1FromPi = 0;
long param2FromPi = 0;
long param3FromPi = 0;

// Purpose
//  Fill the SPI transfer buffer with payload data to be sent to the SPI master on the next SPI byte exchange
// Algorithm
//  always prepares the sendBuffer for the next transfer
//  -> but queues the values to an alternate sendBuffer in case request to change the send buffer came during an ongoing SPI transfer
// Output
//  updates the sendBuffer with values to be shared at the next transfer(s)
//  returns 0 if values queued during without an SPI transfer in progress
//  returns 1 if values queued during an in-progress SPI transfer
unsigned char setDataForPi (char command, signed char TurnVelocity, signed char Throttle, long param1, long param2, long param3 )
{
  unsigned char nextSendBufferSelect;

  // write to digTP29, to facilitate timing measurements via oscilloscope
  digitalWrite(digTP29, HIGH);

  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();
  if ( SPIxferInProgress == 1)
  {
    toggleSendBuffer = 1;
    if (sendBufferSelect == 1)
    {
      nextSendBufferSelect = 0;
    }
    else
    {
      nextSendBufferSelect = 1;
    }
  }
  else
  {
    nextSendBufferSelect = sendBufferSelect;
  }
  
  toSPIBufferByte1.asChar = command;
  toSPIBufferByte2.asSignedChar = TurnVelocity;
  toSPIBufferByte3.asSignedChar = Throttle;
  toSPIBufferLong1.asLong = param1;
  toSPIBufferLong2.asLong = param2;
  toSPIBufferLong3.asLong = param3;

  sendBuffer[nextSendBufferSelect][0] = toSPIBufferByte1.asUnsignedChar;
  sendBuffer[nextSendBufferSelect][1] = toSPIBufferByte2.asUnsignedChar;
  sendBuffer[nextSendBufferSelect][2] = toSPIBufferByte3.asUnsignedChar;
  int i; 
  // queue bytes 4 thru 7 (param1)     
  for (i = 3; i <= 6; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong1.asByte[i-3];
  }   

  // queue bytes 8 thru 11 (param2)     
  for (i = 7; i <= 10; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong2.asByte[i-7];
  }   

  // queue bytes 12 thru 15 (param3)     
  for (i = 11; i <= 14; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong3.asByte[i-11];
  }               
  interrupts();
  digitalWrite(digTP29, LOW);

  if (toggleSendBuffer == 0)
  {
    return 0;   // indicate transfer request accepted, no collision detected
  }
  else
  {
    return 1;   // indicate transfer request accepted, but collision detected so will toggle sendBuffer
  }
}

// Purpose
//  retrieve the latest data received from the SPI master on the most recent SPI byte exchange
unsigned char getLatestDataFromPi ()
{
  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();
  if ( SPIxferInProgress == 0)
  {
    fromSPIBufferByte1.asUnsignedChar = receiveBuffer[0];
    fromSPIBufferByte2.asUnsignedChar = receiveBuffer[1];
    fromSPIBufferByte3.asUnsignedChar = receiveBuffer[2];

    int i; 
    // fetch bytes 4 thru 7 (param1)     
    for (i = 3; i <= 6; i++) 
    {
      fromSPIBufferLong1.asByte[i-3] = receiveBuffer[i];
    }   

    // fetch bytes 8 thru 11 (param2)     
    for (i = 7; i <= 10; i++) 
    {
      fromSPIBufferLong2.asByte[i-7] = receiveBuffer[i];
    }   

    // fetch bytes 12 thru 15 (param3)     
    for (i = 11; i <= 14; i++) 
    {
      fromSPIBufferLong3.asByte[i-11] = receiveBuffer[i];
    }      

    commandFromPi = fromSPIBufferByte1.asChar;
    TurnVelocityFromPi = fromSPIBufferByte2.asSignedChar;
    ThrottleFromPi = fromSPIBufferByte3.asSignedChar;
    param1FromPi = fromSPIBufferLong1.asLong;
    param2FromPi = fromSPIBufferLong2.asLong;
    param3FromPi = fromSPIBufferLong3.asLong;
    interrupts();
    return 1;   // indicate transfer request accepted
  }
  interrupts();
  return 0; // indicate transfer request rejected
}



/***************************************************************  
 Setup SPI in slave mode (1) define MISO pin as output (2) set
 enable bit of the SPI configuration register 
****************************************************************/ 
                    
void setup (void)
{
  Serial.begin(250000);   // Serial:  0(RX), 1(TX) => use the highest possible rate to minimize drag on the CPU
                          // e.g. https://forum.arduino.cc/index.php?topic=76359.0
                          // e.g. https://www.quora.com/What-is-the-baud-rate-and-why-does-Arduino-have-a-baud-rate-of-9-600

  Serial.println("Initializing SPI related items.");

  // Digital Test Points
  pinMode (digTP28, OUTPUT);
  digitalWrite(digTP28, LOW);
  pinMode (digTP29, OUTPUT);
  digitalWrite(digTP29, LOW);
  
  SPIwdPriorMillis = millis();  // initialize the SPI watchdog time counter

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  // queue some initial values to send to the Pi
  setDataForPi('A', 1, 2, 3, 4, 5);

  // initialize the receive buffer     
  int i; 
  for (i = 0; i <= 14; i++) 
  {
    receiveBuffer[i] = i;
  }   

  // turn on SPI interrupts
  SPI.attachInterrupt();

  Serial.println("SPI initialization complete.");

}  

// Purpose
//  Assess arrival of another byte via SPI
// Algorithm
//  Ignore it, Reset the transfer protocol state machine, or add the latest transferred byte to an 'being catured' buffer
//  trigger on SPI received interrupts per this template
//   https://roboticsbackend.com/raspberry-pi-master-arduino-uno-slave-spi-communication-with-wiringpi/

/***************************************************************  
Algorithm & protocol derived from polled example code spiHandler()
  ie. crom http://robotics.hobbizine.com/raspiduino.html
   Uses the SPIxferIndex variable to keep track current position in the
   incoming data packet and execute accordingly
   State  -> Meaning & Action
   -2     - wait for to receive start byte
   -1     - once received start byte, send an acknowledge byte
   0..13  - exchange payoad bytes
   14     - exchange the last payload byte, queue a final 'end of burst' handshake byte
   15     - exchange & verify final 'end of burst' handshake byte, 
            flag new / valid data received or not
            reset the protocol burst state machine
****************************************************************/
ISR (SPI_STC_vect)
{
    // write to digTP28, to facilitate timing measurements via oscilloscope
    digitalWrite(digTP28, HIGH);

    // reset the SPI listener state machine if the prior transfer has clearly taken too long or was prematurely cancelled
    // ie. this will force the listener to wait for another initial 'start new transfer' SPIxferIndex byte
    // choose threshold based on empirical timing observation by oscilloscope plus a little margin
    if ( (millis() - SPIwdPriorMillis) > 5)
    {
      SPIxferIndex = -2;
      SPIxferInProgress = 0;   // flag to external programs that the receiveBuffer 'can be trusted'
      newSPIdataAvailable = 0; // reset the new data flag, even if prior new data was not read
    }
         
    switch (SPIxferIndex)
    {
    case -2:  // ie. this is potentially the 1st header byte
      SPIwdPriorMillis = millis();   // reset the watchdog timer
      if (SPDR == 's')    // confirmed, we presume this is the 1st header byte
      {
        SPDR = 'a';             // queue an acknowledge byte to be sent on the next transfer
        SPIxferInProgress = 1;  // flag to external programs that the receiveBuffer has WIP / cannot be trusted
        newSPIdataAvailable = 0; // reset the new data flag, even if prior new data was not read
        SPIxferIndex++;
      } 
      break;    
    case -1:  // ie. we presume this is the 2nd header byte
      // => in this case, we have just transferred acknowledge 'a' to the master, and have just received a junk byte
      // Hence do not bother reading the SPDR, because in this protocol, the 2nd header byte contains no information for Mega
      // As a potential improvement, this byte could be used to pass additional data from master to slave
      if (toggleSendBuffer == 1)
      {
        toggleSendBuffer = 0;
        if (sendBufferSelect == 1)
        {
          sendBufferSelect = 0;
        }
        else
        {
          sendBufferSelect = 1;
        }
      }
      SPIxferIndex++;
      SPDR = sendBuffer[sendBufferSelect][SPIxferIndex];   // queue the first byte to transfer from Mega slave to Pi master
      break;
    case 14:   // ie. capture the 15th ie. final payload byte & queue and 'end of burst' acknowledge byte
      receiveBuffer[SPIxferIndex] = SPDR;
      SPIxferIndex++; 
      SPDR = 'Z';   // queue an 'end of burst' handshake byte to be sent on the next transfer
                    // => give confidence (albeit not certainty) that the payload bytes transferred between 'a' and 'z' can be trusted
      break;
    case 15:   // ie. verify that just exchanged what should be a final handshaking byte - 'Z' for 'z'. Reset the state machine.
      if (SPDR == 'z')    // verify that the protocol burst state machines were in sync for both SPI master and slave 
      {
        newSPIdataAvailable = 1;  // as best we can tell, the payload bytes between 'a' and 'z' can be trusted
                                  // flag to external functions that new data is now available
      } else
      {
        errorCountSPIrx++;      // count as an error - we got this far, should have received a finaly handshake byte
        newSPIdataAvailable = 0; // flag to external functions that NO new data is available
      }
      SPDR = 0;               // queue a 'defined value' null byte for the next SPI transfer, whenever that may be
      SPIxferIndex = -2;      // reset the state machine
      SPIxferInProgress = 0;  // flag to external functions that the receiveBuffer should be trustable.
      break;
    default:   // otherwise, receive index 0..13 (ie. payload bytes 1..14) & advance SPIxferIndex to next state
      receiveBuffer[SPIxferIndex] = SPDR;
      SPIxferIndex++; 
      SPDR = sendBuffer[sendBufferSelect][SPIxferIndex];    // queue the next byte to be transferred
    }
    digitalWrite(digTP28, LOW);   
}


void loop ()
{

  if ( newSPIdataAvailable == 1 )
  {
    getLatestDataFromPi ();
    newSPIdataAvailable = 0;  
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");
    Serial.print(commandFromPi);
    Serial.print(", ");
    Serial.print(TurnVelocityFromPi);
    Serial.print(", ");
    Serial.print(ThrottleFromPi);
    Serial.print(", ");
    Serial.print(param1FromPi);
    Serial.print(", ");
    Serial.print(param2FromPi);
    Serial.print(", ");
    Serial.println(param3FromPi);
  }
  else
  {
    Serial.println("-> no new data as expected. Either the most recent transfer failed,       ???");
    Serial.println("   or we happened to check when there was no new data...");
  }
  Serial.println("queuing for PI: p, -50, +13, 248, 399, 425");
  if ( setDataForPi('p', -50, +13, 248, 399, 425) == 1)
  {
    Serial.println("-> detected collision with in-progress transfer / toggled sendBuffer for next transfers...");
  }
  Serial.print(" xfer error count ");
  Serial.println(errorCountSPIrx);
  delay(1000);    

  if ( newSPIdataAvailable == 1 )
  {
    getLatestDataFromPi ();
    newSPIdataAvailable = 0;
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");  
    Serial.print(commandFromPi);
    Serial.print(", ");
    Serial.print(TurnVelocityFromPi);
    Serial.print(", ");
    Serial.print(ThrottleFromPi);
    Serial.print(", ");
    Serial.print(param1FromPi);
    Serial.print(", ");
    Serial.print(param2FromPi);
    Serial.print(", ");
    Serial.println(param3FromPi);

  }
    else
  {
    Serial.println("-> no new data as expected. Either the most recent transfer failed,,       ???");
    Serial.println("   or we happened to check when there was no new data...");
  }
  Serial.println("queuing for PI: q, 33, -87, 13987, 22459, 609942");
  if (setDataForPi('q', 33, -87, 13987, 22459, 609942) == 1)
  {
    Serial.println("-> detected collision with in-progress transfer / toggled sendBuffer for next transfers...");
  }
  Serial.println(); 
  Serial.print(" xfer error count ");
  Serial.println(errorCountSPIrx);
  delay(1000);    

}
