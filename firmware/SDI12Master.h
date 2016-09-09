/**

Electron library for communicating with SDI-12 slaves over RS232/485.

Adapted for Electron Robyn Hannah NRI Electronics June 2016
Currently Copyright NRI Electronics 2016. Can be used for evaluation purposes.

*/
/*
*/


//#ifndef SDI12Master_h
//#define SDI12Master_h

#include "application.h"

// currently SERIAL_7E1 is not available but is coming, when firmware is upgraded this should be correct
#ifndef SERIAL_7E1
#define SERIAL_7E1 SERIAL_8N1
#endif

#define sizeBuffer 100	// size of the SDI-12 transmit/receive buffer in bytes


#define lowByte(w)                     ((w) & 0xFF)
#define highByte(w)                    (((w) >> 8) & 0xFF)
#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/* This library was written using an Electron port as the Tx and Rx for the SDI-12 data
		This requires external hardware to combine the two signals into SDI-12 data
		This will be a half duplex circuit which may require both a txEnablePin, rxEnablePin and biasPin
		These can be defined here, sleep mode for the external circuitry is assumed to be txEnable and
		rxEnable both in the disabled state. The enable state is defined as txEnableState etc and is HIGH or LOW
		The biasPin will be set HIGH during receive and transmit, only dropped in the SDI-12 sleep state
		which is invoked by function void sdi_sleep()
*/
#define txEnablePin	C4
#define rxEnablePin C5
#define biasPin	D2
#define txEnableState HIGH
#define rxEnableState LOW
#define txDisableState LOW
#define rxDisableState HIGH

/* The following definitions are taken from the timing for the SDI-12 specification
	All the values are in milliSeconds
*/
#define breakTimeMin 15		// spec is 12 but a bit extra does no harm
#define markTimeMin 9		// actually 8.33 but I am not OCD
#define maxSensorResponseTime 15	// time after the command is sent
#define maxTotalResponseTime 900	// time for the longest response which could be expectd
#define characterDelay 2	// maximum time to wait for the next character before a fault flagged
#define delayAfterTransmit 2	// a short delay put in after the last character is sent to allow for switching

#define sdiSuccess 0
#define sdiFail 1
#define sdiTimeout 2		// timeout condition
#define sdiParityError 3 	// parity error in reply
#define sdiReplyError 4		// an error detected in the received string

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Electron class library for communicating with SDI-12 slaves over USART
Uses Electron Serial USART and external circuitry to produce SDI-12 signal
Baud rate and parity are fixed at 7E1, am working on Particle's 8E1 is actually 7 data bits
	otherwise I am going to dig into changing that
*/
class SDI12Master
{
public:
	// functions

	void begin(char);
	void idle(void (*)());
	void setDebug(bool);
	char sdi_cmd(char*);
	void sdi_wake(int);
	void sdi_sleep(void);
	// constructor
	SDI12Master(char);	// for some reason compiler likes a parameter here or it fails so use serial port
	~SDI12Master();		//destructor
  // global variables
	bool debugMode = false;
	char serialPort = 1;
	char slaveID = 0;
	unsigned char txBufferIndex = 0;
	unsigned char txBufferLength = 0;
	unsigned char rxBufferIndex = 0;
	unsigned char rxBufferLength = 0;
	char txBuffer[sizeBuffer];
	char rxBuffer[sizeBuffer];
	int availTime;		// filled in to give time befor sensor will have a reading milliSecs
	char sensorCount;	// filled in to give the number of sensors that will be read
	char sensorsRead;	// number of sensors actually returned with each Dn command
	float sensorReadings[12];	// filled with sensor readings from the Dn command


private:

    void sdi_break();
    void sdi_mark();
    void sdi_transmit(char*);
    char sdi_receive();
	void to7E1(char*);
	bool from7E1(char*);
    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
};
//#endif
