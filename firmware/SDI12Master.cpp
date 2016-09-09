/**

Electron library for communicating with SDI-12 slaves over RS232/485.

Adapted for Electron Robyn Hannah NRI Electronics June 2016
Currently Copyright NRI Electronics 2016. Can be used for evaluation purposes.

*/
/*
*/


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "SDI12Master.h"
#include "Serial4/Serial4.h"	// for some reason this has to be in this file, not xxxx.h file
#include "Serial5/Serial5.h"

#undef SPARK_SERIAL2 		// #define once Spark Serial2 is implemented

/* _____GLOBAL VARIABLES_____________________________________________________ */
USARTSerial SDISerial = Serial1; 	///< Pointer to Serial1 class object




/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object using given serial port , SDI-12 slave defaults to ID  0.

@ingroup setup
*/
SDI12Master::SDI12Master(char _port)
{
	serialPort = _port;
	slaveID = 0;
}
/*
Destructor
*/
SDI12Master::~SDI12Master()
{
	Serial.println("the end has come");
}

// sets a print out debug mode
void SDI12Master::setDebug(bool _debug)
{
	debugMode = _debug;
}




/**
Initialize class object.

Sets up the serial port using 1200, E,7,1 which is SDI-12 standard

*/
void SDI12Master::begin(char _serialPort)
{
	txBufferIndex = 0;
	txBufferLength = 0;
	rxBufferIndex = 0;
	rxBufferLength = 0;

	switch(serialPort){
		case 1:
			SDISerial = Serial1;
			pinMode(TX,OUTPUT);
			digitalWrite(TX,HIGH);		// output is inverted, will put low on SDI-12 line
			break;
		/*    case 2:
				SDISerial = Serial2;
				break;	// to use Serial 2 would need to #include "Serial2/Serial2.h" at top*/
		case 4:
			SDISerial = Serial4;
			pinMode(C3,OUTPUT);
			digitalWrite(C3,HIGH);		// output is inverted, will put low on SDI-12 line
			break;
		case 5:
			SDISerial = Serial5;
			pinMode(C1,OUTPUT);
			digitalWrite(C1,HIGH);		// output is inverted, will put low on SDI-12 line
			break;
		default:
			SDISerial = Serial1;		// Default to Serial1 for Electron
			pinMode(TX,OUTPUT);
			digitalWrite(TX,HIGH);		// output is inverted, will put low on SDI-12 line
			break;
	}
	pinMode(txEnablePin,OUTPUT);
	digitalWrite(txEnablePin,txEnableState);
	pinMode(rxEnablePin,OUTPUT);
	digitalWrite(rxEnablePin,rxDisableState);
	pinMode(biasPin,OUTPUT);
	digitalWrite(biasPin,HIGH);
	SDISerial.begin(1200,SERIAL_7E1);	// SERIAL_7E1 not implemented in firmware yet, goes to 8N1 with cluge
}


/**
Set idle time callback function (cooperative multitasking).

This function gets called in the idle time between transmission of data
and response from slave. Do not call functions that read from the serial
buffer that is used by SDI12Master. Use of i2c/TWI, 1-Wire, other
serial ports, etc. is permitted within callback function.

@see SDI12Master::ModbusMasterTransaction()
*/
void SDI12Master::idle(void (*idle)())
{
	_idle = idle;
}

/* Function turns bytes - 8 bits -  in a string into a 7 bit data and even parity setting in the msb
		Assumes the msb of c is not set, ie 7 bit data already, if it is set then the routine
		is pointless as it is trying to convert 7 bit characters for transmission by 8N1
		Routine also assumes the string is null terminated     */
void SDI12Master::to7E1(char* ptrC)
{
	char lookup[8] = {0,1,1,0,1,0,0,1};
	char p;
	if(debugMode)Serial.println("Changing the string to 7E1");
	while(*ptrC){
		p = (*ptrC ^ (*ptrC>>4)) & 0x0f;
		if(lookup[p&0x07] ^ (p>>3)){
			*ptrC = *ptrC | 0x80;		// set parity
		}
		ptrC++;
	}
}

/* Reverse of the above, it is taking in characters which are 7E1 and converting them to 7 bit
		characters in the string, can optionally check for parity errors on the characters
		using the msb of the character as the E parity bit*/
bool SDI12Master::from7E1(char* ptrC)
{
	char lookup[8] = {0,1,1,0,1,0,0,1};
	char p,C;
	bool result = false;
	if(debugMode)Serial.println("Translating back to characters from 7E1");
	while(*ptrC){
		C = *ptrC & 0x7f;		// clear parity
		// find out if parity should have been set on the character
		p = (C ^ (C>>4)) & 0x0f;
		if(lookup[p&0x07] ^ (p>>3)){
			if((*ptrC & 0x80)==0x80)result = true;
		}
		else{
			if((*ptrC & 0x80)== 0x00)result = true;
		}
		*ptrC = C;
		ptrC++;
	}
	return result;	// returns false if there was an error on the string
}


/*
Send a command to the SDI-12 sensor bus
char sdi_cmd(char* _sCmd)
The _sCmd string must be zero terminated and must be the complete command
	ie slaveID|Function|!
	return is a char* to the rxBuffer containing the immediate response
	If the command is a start measurement M then the function parses the response to
	obtain the time before the measurement will be available (milliseconds) and the number of sensors
	that will be read. These are sent to the public variables int availTime; and char sensorCount;
	Returns a result of sdiSuccess (0) if all OK else sdiFail (1) or sdiTimeout (2)
*/
char SDI12Master::sdi_cmd(char* _sCmd)
{
	char index,cnt,i;
	char* ptr;
	char* xptr;
	float fvalue[5];
	/* need to create a buffer which is really 7E1 characters, Electron can only
		handle 8N1 at present */
	strcpy(&txBuffer[0],_sCmd);
	if(txBuffer[0] == '?')slaveID = '?';
	else slaveID = txBuffer[0] - 0x30;	 // obviously only works for addresses 0 -> 9
	if(SERIAL_7E1 == SERIAL_8N1){
		// someday there will be a SERIAL_7E1 available, till then cluge it
		to7E1(&txBuffer[0]);
	}
	char result = sdiSuccess;
	sdi_break();	// send break
	sdi_transmit(&txBuffer[0]);
	digitalWrite(rxEnablePin,rxEnableState);
	digitalWrite(txEnablePin,txDisableState);
	result = sdi_receive();
	if(debugMode)Serial.printlnf("Received string %s : Parity %i",&rxBuffer[0],result);
	_sCmd++;
	switch(*_sCmd){
		case 'M':
			if(result == sdiSuccess){
				// just check that the aM! bit was not included
				if(xptr = strstr(&rxBuffer[0],"M!")){
					xptr = xptr + 2;
				}
				else{
					xptr = strchr(&rxBuffer[0],(slaveID+0x30));
				}
				sensorCount = *(xptr+4) - 0x30;
				if((sensorCount < 1) || (sensorCount > 9))result = sdiReplyError;
				*(xptr+4) = 0;
				xptr++;
				availTime = atoi(xptr);
				*(xptr+3) = sensorCount + 0x30;	// restore for posterity
			}
			break;
		case 'D':
			if(result == sdiSuccess){
				cnt = 0;
				_sCmd++;
				index = (*_sCmd - 0x30); 	// 0, 1 etc
				ptr = &rxBuffer[0];
                while(*ptr){
                    while((*ptr != '+') && (*ptr != '-') && (*ptr != 0))ptr++;		// go to first sign character
					if((*ptr=='+') || (*ptr=='-')){
						fvalue[cnt] = atof(ptr);
						cnt++;
						ptr++;	// go look for the next sign
					}
				}
				index = index * cnt;	// starting point on the results table
				for(i=0;i<cnt;i++)sensorReadings[index+i] = fvalue[i];
				sensorsRead = cnt;
			}
			break;
		case 'I':
			// Send identification, result in rxBuffer
			break;
		case '!':
			if(slaveID == '?'){
				// this is an address query
				if(result == sdiSuccess){
					slaveID = rxBuffer[0] - 0x30;	// available to read
				}
			}
			else{
				// acknowledge active, result is in result
			}
			break;
		
	}
	return result;
}


/*
Wake request. Function puts a break on the SDI-12 line for the duration of the delay
*/
void SDI12Master::sdi_wake(int delay)
{
	unsigned long timeStart;
	SDISerial.end();
	digitalWrite(rxEnablePin,rxDisableState);
	digitalWrite(txEnablePin,txEnableState);
	digitalWrite(biasPin,HIGH);
	timeStart = millis();
	switch(serialPort){
		case 1:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,LOW);
			break;
		case 4:
			pinMode(C3,OUTPUT);
			digitalWrite(C3,LOW);
			break;
		case 5:
			pinMode(C1,OUTPUT);
			digitalWrite(C1,LOW);
			break;
		default:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,LOW);
			break;
	}
	while(millis() < timeStart + delay);
	SDISerial.begin(1200,SERIAL_7E1);	// re enable Serial
}

/*
Sleep state request. Function puts both rx and tx into disabled state and drops the bias pin to LOW
 void sdi_sleep(void);
*/
void SDI12Master::sdi_sleep(void)
{
	if(txEnableState == HIGH)digitalWrite(txEnablePin,LOW);
	else digitalWrite(txEnablePin,HIGH);
	if(rxEnableState == HIGH)digitalWrite(rxEnablePin,LOW);
	else digitalWrite(rxEnablePin,HIGH);
	digitalWrite(biasPin,LOW);;
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */
/*


/* Function puts a break on the SDI-12 data line
		Disables the serial port to do so, does not re enable
*/
void SDI12Master::sdi_break(void)
{
	unsigned long timeStart;
	SDISerial.end();
	if(debugMode)Serial.println("Sending a break signal");
	digitalWrite(rxEnablePin,rxDisableState);
	digitalWrite(txEnablePin,txEnableState);
	digitalWrite(biasPin,HIGH);
	timeStart = millis();
	switch(serialPort){
		case 1:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,LOW);
			break;
		case 4:
			pinMode(C3,OUTPUT);
			digitalWrite(C3,LOW);
			break;
		case 5:
			pinMode(C1,OUTPUT);
			digitalWrite(C1,LOW);
			break;
		default:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,LOW);
			break;
	}
	while(millis() < timeStart + breakTimeMin);
	SDISerial.begin(1200,SERIAL_7E1);	// re enable Serial
	SDISerial.flush();
}

/* Function puts a mark on the SDI-12 data line, assumes the break function has disabled Serial
		and all is OK to lower the signal on the TX line
		have determined that the serial line goes to mark state when I begin the serial above
		otherwise there is a glitch after my mark and the begin which upsets things.
		Finding that it is 8 bits plus parity which is odd, not 7 plus parity
		Currently this is not called because the SDISerial.begin sequence in the break routine
		takes approximately 10 mSec to come good, this is about the mark time required
*/
void SDI12Master::sdi_mark()
{
	unsigned long timeStart;
	timeStart = millis();
	switch(serialPort){
		case 1:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,HIGH);
			break;
		case 4:
			pinMode(C3,OUTPUT);
			digitalWrite(C3,HIGH);
			break;
		case 5:
			pinMode(C1,OUTPUT);
			digitalWrite(C1,HIGH);
			break;
		default:
			pinMode(TX,OUTPUT);
			digitalWrite(TX,HIGH);
			break;
	}
	while(millis() < timeStart + markTimeMin);
	SDISerial.begin(1200,SERIAL_7E1);	// re enable Serial
	SDISerial.flush();
	while(SDISerial.available()){
		// I am not positive that the flush really clears the receive buffer so do it anyway
		SDISerial.read();
	}

}
void SDI12Master::sdi_transmit(char* ptr)
{
	int bytesW;
	if(debugMode)Serial.printlnf("Transmitting %s", ptr);
	while(*ptr != 0){
		bytesW = SDISerial.write(*ptr);
		SDISerial.flush();		// does not work properly but does wait for the current character to leave
		//if(debugMode)Serial.printlnf("Byte written %i %x",bytesW,*ptr);
		ptr++;
	}
	delay(delayAfterTransmit);
	//digitalWrite(rxEnablePin,rxEnableState);
	//digitalWrite(txEnablePin,txDisableState);
}
char SDI12Master::sdi_receive(void)
{
	/* This will timeout after maxSensorResponseTime if a response has not happened
		and after maxTotalResponseTime even if it is still receiving data	*/
	unsigned long timeStart;
	char read;
	bool resEnd = false;
	bool resStart = false;
	int cnt = 0;

	//digitalWrite(rxEnablePin,rxEnableState);
	//digitalWrite(txEnablePin,txDisableState);
	rxBuffer[0] = 0;
	//slaveID = (txBuffer[0]&0x7f)-0x30;
	timeStart = millis();
	//digitalWrite(rxEnablePin,rxEnableState);
	//digitalWrite(txEnablePin,txDisableState);
	if(debugMode)Serial.printlnf("Receiving characters - expecting from %x",slaveID);
	if(slaveID == '?'){
		resStart = true;
		rxBufferIndex = 0;
		rxBufferLength = 0;
	}
	while (!resEnd && (millis() < (timeStart + maxTotalResponseTime))){
		if (SDISerial.available()){
			read = SDISerial.read();
			//if(debugMode)Serial.write(read);
			if(((read&0x7f-0x30) == slaveID)  && !resStart){
				// response starts with slave ID or the case of address query it is '?'
				resStart = true;
				rxBufferIndex = 0;
				rxBufferLength = 0;
			}
			if(resStart && (rxBufferIndex < sizeBuffer)){
				rxBuffer[rxBufferIndex] = read;
				//if(debugMode)Serial.write(read);
				rxBufferIndex++;
				rxBuffer[rxBufferIndex] = 0;
			}
			if(SERIAL_7E1 == SERIAL_8N1){
				if((read == 0x0a) && (rxBuffer[rxBufferIndex-2] == 0x8d)){
					// 0x0a,0x8d will be the 8N1 equivalent of the 7E1 version of '\n' and '\r'
					resEnd = true;
				}
			}
			else{
				if((read == '\n') && (rxBuffer[rxBufferIndex-2] == '\r')){
					resEnd = true;
				}
			}
		}
	}
	if(resEnd){
		if(SERIAL_7E1 == SERIAL_8N1){
			/* Until SERIAL_7E1 is in firmware we need to do a cluge
					we did get a response, need to translate this back to 7 bit characters and check parity*/
			if(from7E1(&rxBuffer[0])){
				return sdiSuccess;
			}
			else return sdiParityError;
		}
		else{
			return sdiSuccess;	// there does not seem to be any parity checks available in firmware?
		}
	}
	else return sdiTimeout;
}
