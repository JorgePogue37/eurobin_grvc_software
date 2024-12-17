/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ArmController.h
 */
 
 
#ifndef ARMCONTROLLER_H_
#define ARMCONTROLLER_H_


// Standard library
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>


// Specific library
#include "../ServoState/ServoState.h"
#include "../Kinematics/Kinematics.h"


// Constants
#define RX_BUFFER_LENGTH				4096
#define SERVO_RAM_READ_INTERVAL			0.005
#define SERVO_DATA_PACKET_LENGTH_0101	15;
#define SERVO_DATA_PACKET_LENGTH_0201	15;
#define SERVO_DATA_PACKET_LENGTH_0402	27;
#define SERVO_DATA_PACKET_LENGTH_0602	27;

#define ARM_TCP_Z_MAX					0.0F


// Structures


// Namespaces
using namespace std;



class ArmController
{
public:

	/***************** PUBLIC CONSTANTS *****************/


	/***************** PUBLIC VARIABLES *****************/
	
	float tcpPositionReference[3];	// XYZ position reference in meters of tool center point (TCP) referred to shoulder base
	float tcpPosition[3];			// XYZ position in meters of end effector referred to shoulder base
	float tcpOrientation[3];		// Euler ZYX orientation in rad of end effector referred to shoulder base
	float tcpForce[3];				// Force in XYZ axes in [N]
	float tcpTorque[3];				// Torque in XYZ axes in [N·m]
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	ArmController(uint8_t _armID);

	/*
	 * Destructor
	 * */
	virtual ~ArmController();
	
	/*
	 * Initiate arm controller: set the ID's of the servos, sign and offset corrections, joint limits, and set torque ON.
	 */
	int init(int _numArmJoints, uint8_t *_servoIDs, int * _servoModel, int * _signCorrections, float * _offsetCorrections, float * _maxJointLimits, float * _minJointLimits, const string &uartDeviceName);
	
	/*
	 * Enable/disable torque control of the specified servo.
	 */
	void servoTorqueControl(uint8_t servoID, uint8_t controlMode);
	
	/*
	 * Enable/disable torque control of the arm.
	 */
	void armTorqueControl(uint8_t controlMode);

	/*
	 * Get current joints position in [rad]
	 */
	void getJointsPosition(float * q);
	
	/*
	 * Get current Cartesian position in [m]
	 */
	void getCartesianPosition(float * pos);
	
	/*
	 * Move arms to desired Cartesian position. Return non-zero value in case of error (non reachable point).
	 */
	int goToCartesianPosition(float playTime, float * pos_ref);
	int goToCartesianPosition_q2(float playTime, float * pos_ref, float q2);
	int goToCartesianPosition_q2_LinksLength(float playTime, float * pos_ref, float q2, float l1, float l2);
	
	/*
	 * Move the joints of the arm to the desired position in the specified time.
	 */
	int moveJoints(float * q_ref, float playTime);
	
	/*
	 * Move the indicated joint to the desired absolute position in the specified time. Offset and sign corrections
	 * are applied, so the desired goal position is defined with respect to the arm frame.
	 */
	int moveJoint(uint8_t servoID, float position, float playTime);
	
	/*
	 * Move the indicated servo to the desired absolute position in the specified time. Only the sign correction
	 * is applied (no offset correction). This function should be only used for calibration and servo identification.
	 */
	int moveServo(uint8_t servoID, float position, float playTime);

	/*
	 * Terminate the arm servos data thread
	 */
	void endServoThread();
	

private:
	/***************** PRIVATE VARIABLES *****************/
	uint8_t armID;
	int numArmJoints;
	vector<ServoState> armServos;
	vector<uint8_t> servosID;
	vector<int> servosModel;
	vector<int> servosSignCorrection;
	vector<float> servosOffsetAngle;
	vector<float> jointsLimitMax;
	vector<float> jointsLimitMin;
	uint8_t endThreadSignal;
	
	int usbDevice;
	
	Kinematics * armKinematics;
	
	thread	armThread;
	
	
	/***************** PRIVATE METHODS *****************/
	
	/*
	 * Thread function for updating arm servos state
	 */
	void updateArmServosState();
	
	/*
	 * Update servo state from the received data packet. The ID of the corresponding servo is determined from the data packet.
	 */
	int updateServoState(char * dataPacket, int bytesReceived, int servoModel);
	
	/*
	 * Update the reference Cartesian position of the end effector
	 */
	void updateEndEffectorRefPosition(double xref, double yref, double zref);
	
	/*
	 * Limit joint position taking into account mechanical constraints and recommended operation range.
	 */
	float limitServoRotation(uint8_t servoID, float position);
	
	/*
	 * Open and configure the UART interface, giving as output the file descriptor (-1 in case of error)
	 */	
	int createSerialInterface(const string &uartDeviceName, speed_t speed);

	/*
	 * Send data through the serial port
	 */
	int sendData(char * data, uint8_t dataLength, int usbDevice);

	/*
	 * Compute the checksum 1 and checksum 2 of the packet passed as first argument with the specified length.
	 */
	void computeChecksum(char * packet, uint8_t packetSize);
	

};

#endif /* ARMCONTROLLER_H_ */


