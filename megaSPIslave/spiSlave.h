// Facade / Adapter class to configure an Arduino Mega as an SPI slave and exchange byte payloads with an SPI master

#ifndef spiSlave_H
#define spiSlave_H

class spiSlave
{
    public:

    // test points to monitor CPU status via o'scope
    #define digTP28 28  // digital test point #3 (Arduino Mega pin 28) 
    #define digTP29 29  // digital test point #4 (Arduino Mega pin 29) 

    // Default Constructor
    //  Set digital output pins, set initial/default values to transfer to master
    spiSlave();

    //  Purpose
    //      Initialize the SPI port as a slave and enable SPI interrupts
    void enable();

    // Purpose
    //  Fill the SPI transfer buffer with payload data to be sent to the SPI master on the next SPI byte exchange
    unsigned char setDataForPi (char command, signed char TurnVelocity, signed char Throttle, long param1, long param2, long param3 );

    // ToDo come up with a double buffer scheme to avoid the need to test return values from getLatestDataFromPi()
    // ToDo -> also provide a cleaner return value as a structure instead of separate variables
    // Purpose
    //  retrieve the latest data received from the SPI master on the most recent SPI byte exchange
    // Output
    //  returns 1 if request to retreive was accepted, 0 if it was rejected due to collision with in progress transfer
    unsigned char getLatestDataFromPi ();

    volatile unsigned char newSPIdataAvailable = 0; // flag for using functions to determine when new data has been received

    volatile unsigned int errorCountSPIrx = 0;  // let's track apparent xfer failures

    // public methods to retrieve the latest values received from the Pi SPI master
    char getCommandFromPi();
    signed char getTurnVelocityFromPi();
    signed char getThrottleFromPi();
    long getParam1FromPi();
    long getParam2FromPi();
    long getParam3FromPi();

    // Returns maximum SPI observed burst duration in ms, since last cleared. 
    // This should never be higher than the value of maxAllowedSPIburstDuration.
    unsigned char getMaxBurstDuration();

    // Clears an internal register that tracks the maximum oberved SPI burst duration
    void clearMaxBurstDuration();

    // Returns maximum observed delay between SPI bursts in ms, since last cleared. 
    // This can easily and often be higher than the value of maxAllowedSPIburstDuration, depending primarily how often the SPI Master chooses to initiate transfers.
    unsigned long getMaxDelayBetweenBursts();

    // Clears an internal register that tracks the maximum observed delay between SPI bursts.
    void clearMaxDelayBetweenBursts();


    // Purpose
    //  Interrupt handler
    //  Assesses arrival of each byte received via SPI
    // Note
    //  exposed as public to enable visibility to the Arduino ISR (SPI_STC_vect)
    // ==> do NOT call this method, in practice, it is internal to the spiSlave class
    void spiISR();


    private:

    // time threshold in ms to declare a failed burst transfer attempt
    #define maxAllowedSPIburstDuration 30  // assume that something is wrong if a burst of bytes is started but not completed within this time

    // SPI service state machine variables
    volatile unsigned char receiveBuffer[2][15]; // temporary buffer for bytes coming from the SPI master
                                            // double buffer allows setting next burst values even if a current burst is in progress
    volatile unsigned char inProgressReceiveBufferSelect = 0;  // index into receiveBuffer for any current / in-progress SPI transfer. start with the 0th receiveBuffer                                                                
    volatile unsigned char lastCompletedReceiveBufferSelect = 1;  // index to most recently captured receiveBuffer

    volatile unsigned char sendBuffer[2][15]; // temporary buffer for bytes to be sent to SPI master in the next burst
                                            // double buffer allows setting next burst values even if a current burst is in progress
    volatile unsigned char sendBufferSelect = 0;  // currently seleted sendBuffer. start with the 0th sendBuffer                                                                
    volatile unsigned char toggleSendBuffer = 0;  // flag to indicate whether to use the other sendBuffer on the next SPI transfer

    volatile signed char SPIxferIndex = -2;   // state machine tracker- index to byte within incoming SPI burst
                                        // start at -2 to account for 2 header bytes in this protocol

    volatile unsigned long SPIwdPriorMillis;  // watchdog timer for SPI state machine

    volatile unsigned char SPIxferInProgress = 0; // flag using functions to determine whether the receiveBuffer can be trusted, ie. with WIP or a complete set of bytes from a single transfer

    // use unions to simp`lify references to variables by individual bytes or as different types
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
    //  e.g. -> accept pointers in the constuctor initialize() and bind to working values in the spiSlave class
    char commandFromPi = 0;
    signed char TurnVelocityFromPi = 0;
    signed char ThrottleFromPi = 0;
    long param1FromPi = 0;
    long param2FromPi = 0;
    long param3FromPi = 0;

    // Internal register that tracks the maximum oberved SPI burst duration
    volatile unsigned char maxBurstDuration = 0;

    // Internal register that tracks the maximum oberved delay between SPI bursts.
    volatile unsigned long maxDelayBetweenBursts = 0;

};

// Construct an interrupt driven class to manage SPI interactions as a slave
extern spiSlave spiSlavePort;

#endif