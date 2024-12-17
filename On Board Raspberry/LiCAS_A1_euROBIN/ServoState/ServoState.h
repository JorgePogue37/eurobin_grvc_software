/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ServoState.h
 */

#ifndef SERVOSTATE_H_
#define SERVOSTATE_H_


// Standard library
#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>


// Union definition
union Int16ToByteArray
{
	int16_t value;
	uint8_t str[2];
};


using namespace std;


/*
 *
 * THIS CLASS IS SPECIFIC FOR THE HERKULEX DRS-0101, DRS-0201, DRS-0402, DRS-0602 SMART SERVOS
 *
 */
class ServoState
{
public:

	/***************** PUBLIC VARIABLES *****************/
	int servoModel;		// 101, 201, 402 or 602
	uint8_t servoID;	// ID of the servo in the range 0x00 - 0xFD
	float timeStamp;	// Time instant in which the servo state was updated
	float tick;			// Internal time stamp in the range [0, 2.8672] seconds at 11.2 [ms] steps
	float position;		// Servo position in [deg] given by the integrated magnetic encoder
	float position2;	// Servo position in [deg] given by the internal potentiometer
	float positionRef;	// Position reference in [deg]
	float speed;		// Servo shaft speed in [deg/s]
	float pwm;			// Duty cycle of the PWM signal in the range [0, 1]
	float temperature;	// Internal servo temeprature in [ºC] (should be kept monitored for fault tolerance)
	float voltage;		// Internal servo voltage in [V] (should be kept monitored for fault tolerance)
	uint8_t flags;		// LED status, ...
	
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	ServoState(uint8_t _servoID, int _servoModel);
	ServoState(uint8_t _servoID, int _servoModel, int _signCorrection, int _offsetAngle);

	/*
	 * Destructor
	 * */
	virtual ~ServoState();

	/*
	 * Set the sign and offset correction specific to the servo
	 */
	void setJointCorrections(int _signCorrection, int _offsetAngle);
		
	/*
	 * Send a request packet for reading position and speed of the servo through the USB-UART serial interface.
	 */
	void sendStatusUpdateRequest(int usbDevice);
	
	/*
	 * Update servo state from the received data packet. The position, speed and positionRef variables are sign and offset corrected.
	 */
	void updateServoState(char * dataPacket);
	


private:
	/***************** PRIVATE VARIABLES *****************/
	
	int signCorrection;
	float offsetAngle;
	
	
	/***************** PRIVATE METHODS *****************/

	/*
	 * Send data through the serial port
	 *
	 *	Arguments:
	 *		- data: pointer to data to send
	 *		- dataLength: number of bytes to send
	 */
	int sendData(char * data, uint8_t dataLength, int usbDevice);
		
	/*
	 * Compute the checksum 1 and checksum 2 of the packet passed as first argument with the specified length.
	 */
	void computeChecksum(char * packet, uint8_t packetSize);

};

#endif /* SERVOSTATE_H_ */


