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

#include "spiSlave.h"


// placeholder variables to provide interface between the SPI service and the functions using the SPI service
// ToDo -> eventually refactor these for more clean & abstracted interface to SPI service
// signed char TurnVelocityFromMega = 0;
// signed char ThrottleFromMega = 0;

                   
void setup (void)
{
  Serial.begin(250000);   // Serial:  0(RX), 1(TX) => use the highest possible rate to minimize drag on the CPU
                          // e.g. https://forum.arduino.cc/index.php?topic=76359.0
                          // e.g. https://www.quora.com/What-is-the-baud-rate-and-why-does-Arduino-have-a-baud-rate-of-9-600

  Serial.println("Enabling the SPI as a slave.");

  spiSlavePort.enable();

  Serial.println("SPI initialization complete.");

}  

void loop ()
{

  if ( spiSlavePort.getLatestDataFromPi () == 1 )
  {
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");
    Serial.print(spiSlavePort.getCommandFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getTurnVelocityFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getThrottleFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getParam1FromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getParam2FromPi());
    Serial.print(", ");
    Serial.println(spiSlavePort.getParam3FromPi());
  }
  else
  {
    Serial.println("-> no new data as expected. Either the most recent transfer failed,       ???");
    Serial.println("   or we happened to check when there was no new data...");
  }

  Serial.println();
  spiSlavePort.handleCommandsFromPi();
  if (spiSlavePort.getNextSPIxferToPiReserved())
  {
    Serial.println("Started to queue for Pi, but did not since getNextSPIxferToPiReserved() was true.");
  }
  else
  {
    Serial.println("queuing for PI: p, Max burst duration, +13, 248, 399, 425");
    if ( spiSlavePort.setDataForPi('p', spiSlavePort.getMaxBurstDuration(), +13, 248, 399, 425) == 1)
    {
      Serial.println("-> detected collision with in-progress transfer / toggled sendBuffer for next transfers...");
    }
  }
  Serial.println(); 
  Serial.print(" xfer error count, num bursts rejected too long ");
  Serial.print(spiSlavePort.getErrorCountSPIrx());
  Serial.print(", ");
  Serial.println(spiSlavePort.getNumBurstsRejectedTooLong());
  Serial.print(" max SPI burst duration (ms), max delay between SPI bursts (ms) ");
  Serial.print(spiSlavePort.getMaxBurstDuration());
  Serial.print(", ");
  Serial.println(spiSlavePort.getMaxDelayBetweenBursts());    
  delay(1000);    
  Serial.println();


  if ( spiSlavePort.getLatestDataFromPi () == 1 )
  {
    Serial.println("from Pi: command, TurnVelocity, Throttle, param1, param2, param3");  
    Serial.print(spiSlavePort.getCommandFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getTurnVelocityFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getThrottleFromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getParam1FromPi());
    Serial.print(", ");
    Serial.print(spiSlavePort.getParam2FromPi());
    Serial.print(", ");
    Serial.println(spiSlavePort.getParam3FromPi());
  }
    else
  {
    Serial.println("-> no new data as expected. Either the most recent transfer failed,,       ???");
    Serial.println("   or we happened to check when there was no new data...");
  }


  Serial.println();
  spiSlavePort.handleCommandsFromPi();
  if (spiSlavePort.getNextSPIxferToPiReserved())
  {
    Serial.println("Started to queue for Pi, but did not since getNextSPIxferToPiReserved() was true.");
  }
  else
  {
    Serial.println("queuing for PI: q, Max burst duration, -87, 13987, 22459, spiSlavePort.getMaxDelayBetweenBursts()");
    if (spiSlavePort.setDataForPi('q', spiSlavePort.getMaxBurstDuration(), -87, 13987, 22459, (long) spiSlavePort.getMaxDelayBetweenBursts()) == 1) //note: loss of fidelty from casting unsigned long to long...
    {
      Serial.println("-> detected collision with in-progress transfer / toggled sendBuffer for next transfers...");
    }
  }
  Serial.println(); 
  Serial.print(" xfer error count, num bursts rejected too long ");
  Serial.print(spiSlavePort.getErrorCountSPIrx());
  Serial.print(", ");
  Serial.println(spiSlavePort.getNumBurstsRejectedTooLong());
  Serial.print(" max SPI burst duration (ms), max delay between SPI bursts (ms) ");
  Serial.print(spiSlavePort.getMaxBurstDuration());
  Serial.print(", ");
  Serial.println(spiSlavePort.getMaxDelayBetweenBursts());    
  delay(1000);    
  Serial.println();
}
