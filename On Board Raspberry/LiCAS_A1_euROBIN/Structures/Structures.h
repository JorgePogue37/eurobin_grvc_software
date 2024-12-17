/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Structures.h
 */


#ifndef STRUCTURES_H_
#define STRUCTURES_H_


#include "../ArmController/ArmController.h"
#include "../Kinematics/Kinematics.h"
#include "./StructuresInterface.h"



// Structure containing all the variables and class instances shared between the threads
typedef struct
{
	int key;
	int keepAliveWarning;
	int gcsCode;
	bool gcsCodeReceivedFlag;
	bool endFlag;
} THREAD_ARGS;


// Data packet received from the STM32 board through the USART containing IMU data
typedef struct
{
	uint8_t header[2];		// 0xFD 0xFD byte sequence
	int16_t ypr[3];			// Orientation in deg x 10
	int16_t Axyz[3];		// Acceleration in m/s^2 x 10
	int16_t Gxyz[3];		// Gyroscope in deg/s x 10
} __attribute__((packed)) IMU_DATA_PACKET;


// Data packet sent to the STM32 board through the USART for controlling the grippers
typedef struct
{
	char header[3];		// "CMD" character sequence
	uint8_t packetID;
	uint8_t pwm1;
	uint8_t pwm2;
	uint8_t  checksum;
} __attribute__((packed)) COMMAND_PACKET;


// Data packet received from the Ground Control Station through an UDP socket
typedef struct
{
	char header[3];		// "GCS" character sequence
	int code;
} GCS_PACKET;

// Data packet received from the teleoperation node
typedef struct
{
	char header[3];		// "TOP" character sequence
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	int buttons[2];;
} TELEOP_PACKET;


union floatToByteArray
{
	float value;
	uint8_t byteArray[4];
};



#endif

