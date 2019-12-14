#ifndef spiSlave_h
#define spiSlave_h


class spiSlave
{
    public:

    // test points to monitor CPU status via o'scope
    #define digTP28 28  // digital test point #3 (Arduino Mega pin 28) 
    #define digTP29 29  // digital test point #4 (Arduino Mega pin 29) 

    // Constructor
    //  Set digital output pins, set initial/default values to transfer to master
    //  Initialize the SPI port and enable SPI interrupts
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

    // placeholder variables to provide interface between the SPI service and the functions using the SPI service
    // ToDo -> eventually refactor these for more clean & abstracted interface to SPI service
    //  e.g. -> accept pointers in the constuctor initialize() and bind to working values in the spiSlave class
    char commandFromPi = 0;
    signed char TurnVelocityFromPi = 0;
    signed char ThrottleFromPi = 0;
    long param1FromPi = 0;
    long param2FromPi = 0;
    long param3FromPi = 0;



    private:

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

};

#endif