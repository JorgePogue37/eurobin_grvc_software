/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: StructuresInterface.h
 */
 
 
#ifndef STRUCTURES_INTERFACE_H_
#define STRUCTURES_INTERFACE_H_

#define NUM_ARM_JOINTS	2


/****************************** VISION INTERFACE ******************************/

// Data packet received from the vision sensor through an UDP socket
typedef struct
{
	uint8_t header[2];					// "VS" character sequence
	uint8_t packetID;
	float timeStamp;
	float leftArmGoalPosition[3];
	float rightArmGoalPosition[3];
	float leftArmGoalOrientation[3][3];
	float rightArmGoalOrientation[3][3];
	uint8_t checksum;
} VISION_SENSOR_DATA_PACKET;


/****************************** CONTROL INTERFACE ******************************/

typedef struct
{
	uint8_t mode;
	double leftArmGripperPosRef;
	double leftArmJointPosRef[NUM_ARM_JOINTS];
	double leftArmCartesianPosRef[3];
	double leftArmTorqueRef[NUM_ARM_JOINTS];
	double leftArmForceRef[3];
	double rightArmGripperPosRef;
	double rightArmJointPosRef[NUM_ARM_JOINTS];
	double rightArmCartesianPosRef[3];
	double rightArmTorqueRef[NUM_ARM_JOINTS];
	double rightArmForceRef[3];
	double timeStamp;
} ARMS_CONTROL_REFERENCES_DATA_PACKET;


typedef struct
{
	double leftArmJointValues[NUM_ARM_JOINTS];		// Joint values in [rad]
	double rightArmJointValues[NUM_ARM_JOINTS];		// Joint values in [rad]
	double leftTCPCartesianPos[3];					// Tool Center Point Cartesian position in [m]
	double rightTCPCartesianPos[3];					// Tool Center Point Cartesian position in [m]
} ARMS_STATE_PUBLISHER_DATA_PACKET;


#endif


