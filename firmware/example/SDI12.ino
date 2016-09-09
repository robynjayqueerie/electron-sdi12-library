/*

*/

#include "SDI12Master.h"


// instantiate SDI12Master object using serial port 4 - only 1 and 4 are valid
SDI12Master node(4);


void setup()
{
	delay(10000);
	Serial.println("entering setup");
	//node.setDebug(true);
	node.begin(4);	// initialize SDI12Master to use Serial4
}


void loop()
{
	char result;
	char num,reqString[20],cnt;
	Serial.println("hello, another run through the loop");
	Serial.println("Applying power and wait");
	pinMode(B0,OUTPUT);
	digitalWrite(B0,HIGH);
	delay(1000);
	Serial.println("Waking the probes up");
	node.sdi_wake(1000);
	delay(2000);
	Serial.println("Asking for the identity of the probe");
	result = node.sdi_cmd("0I!");
	if(result == sdiSuccess)Serial.printlnf("Probe 0 ID is %s",node.rxBuffer);
	delay(2000);
	Serial.println("Asking the acknowledge active question ");
	if((result = node.sdi_cmd("0!"))== sdiSuccess)Serial.println("Probe with address 0 is active");
	else Serial.println("Probe with address 0 did not respond");
	delay(500);
	if((result = node.sdi_cmd("2!")) == sdiSuccess)Serial.println("Probe with address 2 is active");
	else Serial.println("Probe with address 2 did not respond");
	delay(500);
	Serial.println("Asking for a probe to spill its address ");
	result = node.sdi_cmd("?!");
	if(result == sdiSuccess)Serial.printlnf("Probe with address %i answered",node.slaveID);
	delay(2000);
	Serial.println("Asking for soil moisture measurement ");
	result = node.sdi_cmd("0M!");
	if(result == sdiSuccess){
		Serial.printlnf("Sensor count is %i: Measurement available in %i secs",node.sensorCount,node.availTime);
		delay(node.availTime*1000);
		Serial.println("Reading the measurements ");
		num = 0;	// counts off the number of measurements read
		cnt = 0;	// index for the Dn request
		while((num < node.sensorCount) && (num <10)){
			// a probe can't have more than 10 sensors
			sprintf(&reqString[0],"0D%d!",cnt);
			result = node.sdi_cmd(&reqString[0]);
			if(result != sdiSuccess)break;
			num = num + node.sensorsRead;
			cnt++;
		}
		if(result == sdiSuccess){
			Serial.println("Readings: ");
			for(cnt=0;cnt<node.sensorCount;cnt++){
				Serial.printlnf("%i  %f",cnt,node.sensorReadings[cnt]);
			}
		}
	}
	delay(500);
	Serial.println("Asking for soil temperature measurement ");
	result = node.sdi_cmd("0M2!");
	if(result == sdiSuccess){
		Serial.printlnf("Sensor count is %i: Measurement available in %i secs",node.sensorCount,node.availTime);
		delay(node.availTime*1000);
		Serial.println("Reading the Temperature measurements ");
		num = 0;	// counts off the number of measurements read
		cnt = 0;	// index for the Dn request
		while((num < node.sensorCount) && (num <10)){
			// a probe can't have more than 10 sensors
			sprintf(&reqString[0],"0D%d!",cnt);
			result = node.sdi_cmd(&reqString[0]);
			if(result != sdiSuccess)break;
			num = num + node.sensorsRead;
			cnt++;
		}
		if(result == sdiSuccess){
			Serial.println("Temperature Readings: ");
			for(cnt=0;cnt<node.sensorCount;cnt++){
				Serial.printlnf("%i  %f",cnt,node.sensorReadings[cnt]);
			}
		}
	}
	digitalWrite(B0,LOW);
	node.sdi_sleep();
	delay(10000);
}
