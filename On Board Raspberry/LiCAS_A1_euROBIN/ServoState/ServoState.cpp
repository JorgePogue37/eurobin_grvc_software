/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ServoState.cpp
 */

#include "ServoState.h"



/*
 * Constructor
 * */
ServoState::ServoState(uint8_t _servoID, int _servoModel)
{
	// Set the ID and model of the servo
	servoID = _servoID;
	servoModel = _servoModel;
	
	// Init the sign correction and offset angle
	signCorrection = 1;
	offsetAngle = 0;
}

/*
 * Constructor - Overloaded
 * */
ServoState::ServoState(uint8_t _servoID, int _servoModel, int _signCorrection, int _offsetAngle)
{
	// Set the ID and model of the servo
	servoID = _servoID;
	servoModel = _servoModel;
	
	// Set the sign correction and offset angle
	signCorrection = _signCorrection;
	offsetAngle = _offsetAngle;
}


/*
 * Destructor
 * */
ServoState::~ServoState()
{
}


/*
 * Set the sign and offset correction specific to the servo
 */
void ServoState::setJointCorrections(int _signCorrection, int _offsetAngle)
{
	// Set the sign correction and offset angle
	signCorrection = _signCorrection;
	offsetAngle = _offsetAngle;
}
	
	
/*
 * Send a request packet for reading:
 * 	Voltage (RAM addres 54)
 *	Temperature (55), Tick (57)
 *	Calibrated Position (58-59)
 *	Absolute Position (59-60)
 *	Differential Position (61-62)
 *	PWM (64-65)
 *	Absolute 2nd Position (66-67)
 *	Absolute Goal Position (RAM address 68-69)
 */
void ServoState::sendStatusUpdateRequest(int usbDevice)
{
	char packet[16];
	

	// Set the fields of the packet
	packet[0] = 0xFF;		// Header
	packet[1] = 0xFF;
	packet[2] = 0x09;		// Packet size
	packet[3] = servoID;	// Servo ID
	packet[4] = 0x04;		// RAM read
	packet[7] = 54;			// Initial address (Calibrated position RAM register)
	packet[8] = 16;			// Bytes to read (until Absolute Goal Position)
	
	// Compute the checksum
	computeChecksum((char*)packet, packet[2]);
	
	// Send the packet
	sendData((char*)packet, packet[2], usbDevice);
}
	
	
/*
 * Update servo state from the received data packet. The position, speed and positionRef variables are sign and offset corrected.
 */
void ServoState::updateServoState(char * dataPacket)
{
	Int16ToByteArray aux;
	uint8_t byte1 = 0;
	uint8_t byte2 = 0;
	uint8_t servoID = 0;
	
	
	
	if(servoModel == 402 || servoModel == 602)
	{
		servoID = dataPacket[3];
	
		voltage = 10*dataPacket[9];
		temperature = dataPacket[10];
		tick = 0.0112*dataPacket[12];
		
		byte1 = (uint8_t)dataPacket[15];
		byte2 = (uint8_t)dataPacket[16];
		position = 0.02778*(byte1 + 256*byte2 - 16384);
				
		// Get speed value codified as an int16_t
		aux.str[1] = dataPacket[18];
		aux.str[0] = dataPacket[17];
		speed = 0.62*aux.value;
				
		// Get PWM value codified as an int16_t
		aux.str[1] = dataPacket[20];
		aux.str[0] = dataPacket[19];
		pwm = 9.76562e-4*aux.value;
				
		byte1 = (uint8_t)dataPacket[21];
		byte2 = (uint8_t)dataPacket[22];
		position2 = 0.163*(byte1 + 256*byte2 - 1024);
		byte1 = (uint8_t)dataPacket[23];
		byte2 = (uint8_t)dataPacket[24];
		positionRef = 0.02778*(byte1 + 256*byte2 - 16384);
	}
	else if(servoModel == 101 || servoModel == 201)
	{
		voltage = 0.074*dataPacket[9];
		temperature = 0.381166*dataPacket[10];
		tick = 0.0112*dataPacket[12];
			
		byte1 = (uint8_t)dataPacket[15];
		byte2 = (uint8_t)dataPacket[16];
		position = 0.325*(byte1 + 256*byte2 - 512);
				
		// Get speed value codified as an int16_t
		aux.str[1] = dataPacket[18];
		aux.str[0] = dataPacket[17];
		speed = 29.09*aux.value;
				
		// Get PWM value codified as an int16_t
		aux.str[1] = dataPacket[20];
		aux.str[0] = dataPacket[19];
		pwm = 9.76562e-4*aux.value;
		
		byte1 = (uint8_t)dataPacket[23];
		byte2 = (uint8_t)dataPacket[24];
		positionRef = 0.325*(byte1 + 256*byte2 - 512);
	}
		
	// Apply sign correction
	position = signCorrection*position - offsetAngle;
	speed = signCorrection*speed;
	pwm = signCorrection*pwm;
	position2 = signCorrection*position2 - offsetAngle;
	positionRef = signCorrection*positionRef - offsetAngle;
}


/*
 * Send data through the serial port
 */
int ServoState::sendData(char * data, uint8_t dataLength, int usbDevice)
{
	ssize_t bytesSent = 0;
	int error = 0;
	
	
	// Send data
	bytesSent = write(usbDevice, data, dataLength);
	if(bytesSent <= 0)
	{
		error = 1;
		cout << "ERROR: [in ServoState::sendData] could not send any data" << endl;
	}
	else if(bytesSent < dataLength)
	{
		error = 2;
		cout << "ERROR: [in ServoState::sendData] could not send all data" << endl;
	}

	
	return error;
}


/*
 * Compute the checksum 1 and checksum 2 of the packet passed as first argument with the specified length.
 */
void ServoState::computeChecksum(char * packet, uint8_t packetSize)
{
	uint8_t checksum1 = 0;
	uint8_t checksum2 = 0;
	uint8_t checksumTemp = 0;
	
	
	// Compute checksum-1
	checksumTemp = packet[2] ^ packet[3] ^ packet[4];
	for(uint8_t n = 7; n < packetSize; n++)
		checksumTemp ^= packet[n];
	checksum1 = checksumTemp & 0xFE;
	
	// Set checksum-1 to packet
	packet[5] = checksum1;
	
	// Compute checksum-2
	checksum2 = (~checksumTemp) & 0xFE;
	
	// Set checksum-2
	packet[6] = checksum2;
}



