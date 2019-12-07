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
volatile unsigned char sendBuffer[15]; // temporary buffer for bytes to be sent to SPI master in the next burst                        

volatile signed char SPIxferIndex = -2;   // state machine tracker- index to byte within incoming SPI burst
                                    // start at -2 to account for 2 header bytes in this protocol

volatile unsigned long SPIwdPriorMillis;  // watchdog timer for SPI state machine

volatile unsigned char SPIxferInProgress = 0; // flag using functions to determine whether the receiveBuffer can be trusted, ie. with WIP or a complete set of bytes from a single transfer

volatile unsigned char newSPIdataAvailable = 0; // flag for using functions to determine when new data has been received


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

union longUnion toSpiBufferLong1, toSpiBufferLong2, toSpiBufferLong3 ;
union longUnion fromSpiBufferLong1, fromSpiBufferLong2, fromSpiBufferLong3 ;

// placeholder variables to provide interface between the SPI service and the functions using the SPI service
// ToDo -> eventually refactor these for more clean & abstracted interface to SPI service
signed char TurnVelocityFromMega;
signed char ThrottleFromMega;

char commandFromPi;
signed char TurnVelocityFromPi;
signed char ThrottleFromPi;
long param1FromPi;
long param2FromPi;
long param3FromPi;

// Purpose
//  Fill the SPI transfer buffer with data to be sent to the SPI master on the next SPI byte exchange
unsigned char setDataForPi (char command, signed char TurnVelocity, signed char Throttle, long param1, long param2, long param3 )
{
  // write to digTP29, to facilitate timing measurements via oscilloscope
  digitalWrite(digTP29, HIGH);

  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();
  if ( SPIxferInProgress == 0)
  {
    toSPIBufferByte1.asChar = command;
    toSPIBufferByte2.asSignedChar = TurnVelocity;
    toSPIBufferByte3.asSignedChar = Throttle;
    toSPIBufferLong1.asLong = param1;
    toSPIBufferLong2.asLong = param2;
    toSPIBufferLong3.asLong = param3;

    sendBuffer[0] = toSPIBufferByte1.asUnsignedchar;
    sendBuffer[1] = toSPIBufferByte2.asUnsignedchar;
    sendBuffer[2] = toSPIBufferByte3.asUnsignedchar;
    int i; 
    // queue bytes 4 thru 7 (param1)     
    for (i = 3; i <= 6; i++) 
    {
      sendBuffer[i] = toSpiBufferLong1.asByte[i-3];
    }   

    // queue bytes 8 thru 11 (param2)     
    for (i = 7; i <= 10; i++) 
    {
      sendBuffer[i] = toSpiBufferLong2.asByte[i-7];
    }   

    // queue bytes 12 thru 15 (param3)     
    for (i = 11; i <= 14; i++) 
    {
      sendBuffer[i] = toSpiBufferLong3.asByte[i-11];
    }               
    interrupts();
    digitalWrite(digTP29, LOW);
    return 1;   // indicate transfer request accepted
  }
  interrupts();
  digitalWrite(digTP29, LOW);
  return 0; // indicate transfer request rejected
}

// Purpose
//  retrieve the latest data received from the SPI master on the most recent SPI byte exchange
unsigned char getLatestDataFromPi ()
{
  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();
  if ( SPIxferInProgress == 0)
  {
    fromSPIBufferByte1.unsignedChar = receiveBuffer[0];
    fromSPIBufferByte2.unsignedChar = receiveBuffer[1];
    fromSPIBufferByte3.unsignedChar = receiveBuffer[2];

    int i; 
    // fetch bytes 4 thru 7 (param1)     
    for (i = 3; i <= 6; i++) 
    {
      fromSpiBufferLong1.asByte[i-3] = receiveBuffer[i];
    }   

    // fetch bytes 8 thru 11 (param2)     
    for (i = 7; i <= 10; i++) 
    {
      fromSpiBufferLong2.asByte[i-7] = receiveBuffer[i];
    }   

    // fetch bytes 12 thru 15 (param3)     
    for (i = 11; i <= 14; i++) 
    {
      fromSpiBufferLong3.asByte[i-11] = receiveBuffer[i];
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
         the SPIxferIndex   
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
      SPIxferIndex++;
      SPDR = sendBuffer[SPIxferIndex];   // queue the first byte to transfer from Mega slave to Pi master
      break;
    case 14:   // ie. capture the 15th ie. final payload byte & reset the state machine.
      receiveBuffer[SPIxferIndex] = SPDR;
      SPIxferIndex = -2;
      SPIxferInProgress = 0;  // flag to external functions that the receiveBuffer can be trusted
      newSPIdataAvailable = 1; // flag to external functions that new data is now available
      digitalWrite(digTP28, LOW);   
      break;
    default:   // otherwise, receive index 0..13 (ie. payload bytes 1..14) & advance SPIxferIndex to next state
      receiveBuffer[SPIxferIndex] = SPDR;
      SPIxferIndex++; 
      SPDR = sendBuffer[SPIxferIndex];    // queue the next byte to be transferred
}


void loop (void)
{

  if ( newSPIdataAvailable == 1 )
  {
    getLatestDataFromPi ();
    newSPIdataAvailable = 0;  
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");
    Serial.print(commandFromPi);
    Serial.print(', ');
    Serial.print(TurnVelocityFromPi);
    Serial.print(', ');
    Serial.print(ThrottleFromPi);
    Serial.print(', ');
    Serial.print(param1FromPi);
    Serial.print(', ');
    Serial.print(param2FromPi);
    Serial.print(', ');
    Serial.println(param3FromPi);
  }
  Serial.println("queuing for PI: p, -50, +13, 248, 399, 425");
  setDataForPi('p', -50, +13, 248, 399, 425);
  delay(3000);    

  if ( newSPIdataAvailable == 1 )
  {
    getLatestDataFromPi ();
    newSPIdataAvailable = 0;
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");  
    Serial.print(commandFromPi);
    Serial.print(', ');
    Serial.print(TurnVelocityFromPi);
    Serial.print(', ');
    Serial.print(ThrottleFromPi);
    Serial.print(', ');
    Serial.print(param1FromPi);
    Serial.print(', ');
    Serial.print(param2FromPi);
    Serial.print(', ');
    Serial.println(param3FromPi);

  }
  Serial.println("queuing for PI: q, 33, -87, 13987, 22459, 609942");
  setDataForPi('q', 33, -87, 13987, 22459, 609942);
  Serial.println(); 
  delay(3000);    

}
