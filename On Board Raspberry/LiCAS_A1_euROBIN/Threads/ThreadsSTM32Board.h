/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ThreadsSTM32Board.h
 */
 
 
#ifndef THREADSSTM32BOARD_H_
#define THREADSSTM32BOARD_H_

// Standard library
#include <iostream>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>


// Specific library
#include "../Structures/Structures.h"
#include "../Constants/Constants.h"


// Constant definition
#define RX_BUFFER_SIZE		1024


// Structure definition

typedef struct
{
	uint8_t header[3];				// "RET"
	uint8_t servoID;
	uint8_t packetID;
	int16_t servoPosition;			// Encoder position in deg x 100
	int16_t servoSpeed;				// Servo speed (motor speed / 100) in [deg/s] x 100
	int16_t motorCurrent;			// Motor current in [A] x 100
	int16_t motorTorque;			// Motor torque in [mNm]
	int16_t calibration;			// ADC calibration value [0, 4096]
	uint8_t checksum;
} __attribute__((packed)) RET_PACKET;


/*
 * Thread function for micro-controller board data reception. Data packets containing the measurements
 * of joints deflection provided by the compliant torque sensors are received through the serial interface.
 */
static void * STM32BoardThreadFunction(void * args)
{
	THREAD_ARGS * threadArgs = (THREAD_ARGS*)args;
	IMU_DATA_PACKET * imuDataPacket = NULL;
	uint8_t header[2] = {0xFD, 0xFD};
	uint8_t bufferRx[RX_BUFFER_SIZE];
	uint8_t buffer[RX_BUFFER_SIZE];
	uint8_t bufferAux[RX_BUFFER_SIZE];
	uint8_t tempBuffer[RX_BUFFER_SIZE];
	ssize_t bytesRead = 0;
	ssize_t bytesReceived = 0;
	unsigned int k = 0;
	unsigned int n = 0;
	uint8_t checksum = 0;
	uint8_t packetCorrect = 0;
	int error = 0;
	
	uint8_t rxState = 1;
	unsigned int headerIndex = 0;
	bool headerFound = false;
	bool packetReceived = false;
	bool bufferOverflow = false;
	extern struct timeval t0;
	struct timeval t1;
	float t = 0;
	struct timeval tini;
	struct timeval tend;
	float delta_t = 0;
	const float updatePeriod = 0.00667;
	
	// IMU variables
	float roll = 0, pitch = 0, yaw = 0;
	float Ax = 0, Ay = 0, Az = 0;
	float Gx = 0, Gy = 0, Gz = 0;
	
	// Data file
	ofstream outDataFile;
	
	
	cout << "IMU data thread started" << endl;
	
	// Open data file
	outDataFile.open("IMU_DataFile.txt");
	
	// Init data acquisition loop
	while(error == 0 && threadArgs->endFlag == 0)
	{
		// Get time stamp at the begining of the loop
		gettimeofday(&tini, NULL);
		
		// Initialize variables of the state machine
		bytesReceived = 0;
		rxState = 1;
		headerFound = false;
		packetReceived = false;
		
		while(packetReceived == false && error == 0 && threadArgs->endFlag == false)
		{
			switch (rxState)
			{
				case 1:
					// Wait the reception of a new data packet
					bytesRead = read(threadArgs->usbDeviceSTM32Board, (char*)bufferRx, RX_BUFFER_SIZE - 1);
					if(bytesRead > 0)
					{
						if(bytesReceived + bytesRead >= RX_BUFFER_SIZE)
						{
							bufferOverflow = true;
							bytesReceived = 0;
							headerFound = false;
							packetReceived = false;
							cout << "WARNING: [in imuDataThread] buffer overflow" << endl;
						}
						else
						{
							// Copy the received data into the buffer
							for(k = 0; k < bytesRead; k++)
							{
								buffer[k + bytesReceived] = bufferRx[k];
								// printf("%d ", bufferRx[k]);
							}
							
							bytesReceived += bytesRead;
						}
					}
					else
					{
						// Wait 50 us
						usleep(50);
					}

					// Decide which is the next state
					if(bytesRead <= 0 || bytesReceived < 2 || bufferOverflow == true)
					{
						rxState = 1;			// Wait the reception of data
						bufferOverflow = false;
					}
					else
					{
						if(headerFound == false)
							rxState = 2;	// After receiving new data, check if header is found
						else
							rxState = 3;	// Header was already found. Now check if packet is completely received.
					}
						
					break;
				
				
				case 2:
					// Look for packet header
					headerFound = false;
					for(k = 0; k < bytesReceived - 1 && headerFound == false; k++)
					{
						if(buffer[k] == header[0] && buffer[k+1] == header[1])
						{
							headerFound = true;
							headerIndex = k;
						}
					}
					
					// Decide which is the next state
					if(headerFound == false)
						rxState = 1;	// Wait the reception of new data in the next state
					else
						rxState = 3;	// Check if the packet has been completely received
					
					break;
			
			
				case 3:		
					// Check if the packet has been fully received
					if(bytesReceived - headerIndex >= sizeof(IMU_DATA_PACKET) - 1)
						packetReceived = true;
					else
						packetReceived = false;
					
					// Decide which is the next state
					if(packetReceived == false)
						rxState = 1;	// Wait the reception of new data in the next state
				
					break;
			}
		}
		
		// Extract the data packet
		for(k = 0; k < sizeof(IMU_DATA_PACKET); k++)
			tempBuffer[k] = buffer[k + headerIndex];
		
		imuDataPacket = (IMU_DATA_PACKET*)tempBuffer;
		if(strncmp((char*)imuDataPacket->header, (char*)header, 2) == 0)
			packetCorrect = 1;
		else
			cout << "WARNING: [in imuDataThread] packet header not recognized" << endl;
		
		// Send the control references to the servos if packet is correct
		if(packetCorrect == 1)
		{	
			gettimeofday(&t1, NULL);
			t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec);
			
			// printf("----");
			
			// Update data on structure
			roll = -0.1*imuDataPacket->ypr[2];
			pitch = -0.1*imuDataPacket->ypr[1];
			yaw = -0.1*imuDataPacket->ypr[0];
			// printf("RPY [deg] = {%.1f, %.1f, %.1f}\n", roll, pitch, yaw);
			
			Ax = 0.1*imuDataPacket->Axyz[0];
			Ay = 0.1*imuDataPacket->Axyz[1];
			Az = 0.1*imuDataPacket->Axyz[2];
			// printf("Axyz [m/s^2] = {%.2f, %.2f, %.2f}\n", Ax, Ay, Az);
			
			Gx = 0.1*imuDataPacket->Gxyz[0];
			Gy = 0.1*imuDataPacket->Gxyz[1];
			Gz = 0.1*imuDataPacket->Gxyz[2];
			// printf("Gxyz [deg/s] = {%.1f, %.1f, %.1f}\n", Gx, Gy, Gz);
			// printf("------ \n");
			
			threadArgs->imuRPY[0] = roll;
			threadArgs->imuRPY[1] = pitch;
			threadArgs->imuRPY[2] = yaw;
			
			threadArgs->imuAxyz[0] = Ax;
			threadArgs->imuAxyz[1] = Ay;
			threadArgs->imuAxyz[2] = Az;
			
			threadArgs->imuGxyz[0] = Gx;
			threadArgs->imuGxyz[1] = Gy;
			threadArgs->imuGxyz[2] = Gz;
			
			// Save data on file
			gettimeofday(&t1, NULL);
			t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec);
			
			outDataFile << t << "\t";
			outDataFile << roll << "\t" << pitch << "\t" << yaw << "\t";
			outDataFile << Ax << "\t" << Ay << "\t" << Az << "\t";
			outDataFile << Gx << "\t" << Gy << "\t" << Gz << "\t";
			outDataFile << endl;
		}
		else
			cout << "Packet error" << endl;
		
		// Get time stamp at the end of the loop
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
		
		// printf("Loop time: %.1f ms\n", 1e3*delta_t);
		// Sleep 1 ms
		if(delta_t > updatePeriod)
		{
			cout << "WARNING: [in IMU_Data_Thread] update rate below " << 1/updatePeriod << " Hz" << endl;
		}
		else
			usleep((useconds_t)(1e6*(updatePeriod - delta_t)));
	}
		
	/******************************** THREAD LOOP END ********************************/
		
	cout << "IMU data thread terminated" << endl;
	
	return 0;
}



/*
 * Thread for receiving feedback from the MHD actuator
 */
static void * servoMHDDataThreadFunction(void * args)
{
	THREAD_ARGS * threadArgs = (THREAD_ARGS*)args;
	RET_PACKET * servoDataPacket = NULL;
	char header[3] = {'R', 'E', 'T'};
	uint8_t bufferRx[RX_BUFFER_SIZE];
	uint8_t buffer[RX_BUFFER_SIZE];
	uint8_t bufferAux[RX_BUFFER_SIZE];
	uint8_t tempBuffer[RX_BUFFER_SIZE];
	ssize_t bytesRead = 0;
	ssize_t bytesReceived = 0;
	unsigned int k = 0;
	unsigned int n = 0;
	uint8_t checksum = 0;
	uint8_t packetCorrect = 0;
	int error = 0;
	
	uint8_t rxState = 1;
	unsigned int headerIndex = 0;
	bool headerFound = false;
	bool packetReceived = false;
	bool bufferOverflow = false;
	extern struct timeval t0;
	struct timeval t1;
	float t = 0;
	struct timeval tini;
	struct timeval tend;
	float delta_t = 0;
	
	float servoPosition = 0;
	float servoPosition_1 = 0;
	float servoSpeed = 0;
	float motorCurrent = 0;
	float motorTorque = 0;
	float t_1 = 0;
	
	
	cout << "MHD Servo data thread started" << endl;
	
	// Init data acquisition loop
	while(error == 0 && threadArgs->endFlag == 0)
	{
		// Get time stamp at the begining of the loop
		gettimeofday(&tini, NULL);
		
		// Initialize variables of the state machine
		bytesReceived = 0;
		rxState = 1;
		headerFound = false;
		packetReceived = false;
		
		while(packetReceived == false && error == 0 && threadArgs->endFlag == false)
		{
			switch (rxState)
			{
				case 1:
					// Wait the reception of a new data packet
					bytesRead = read(threadArgs->usbDeviceSTM32BoardMHD, (char*)bufferRx, RX_BUFFER_SIZE - 1);
					
					if(bytesRead > 0)
					{
						if(bytesReceived + bytesRead >= RX_BUFFER_SIZE)
						{
							bufferOverflow = true;
							bytesReceived = 0;
							headerFound = false;
							packetReceived = false;
							cout << "WARNING: [in servoDataThread] buffer overflow" << endl;
						}
						else
						{
							// Copy the received data into the buffer
							for(k = 0; k < bytesRead; k++)
							{
								buffer[k + bytesReceived] = bufferRx[k];
							}
							
							bytesReceived += bytesRead;
						}
					}
					else
					{
						// Wait 50 us
						usleep(50);
					}

					// Decide which is the next state
					if(bytesRead <= 0 || bytesReceived < 3 || bufferOverflow == true)
					{
						rxState = 1;			// Wait the reception of data
						bufferOverflow = false;
					}
					else
					{
						if(headerFound == false)
							rxState = 2;	// After receiving new data, check if header is found
						else
							rxState = 3;	// Header was already found. Now check if packet is completely received.
					}
						
					break;
				
				
				case 2:
					// Look for packet header
					headerFound = false;
					for(k = 0; k < bytesReceived - 2 && headerFound == false; k++)
					{
						if(buffer[k] == header[0] && buffer[k+1] == header[1] && buffer[k+2] == header[2])
						{
							headerFound = true;
							headerIndex = k;
						}
					}
					
					// Decide which is the next state
					if(headerFound == false)
						rxState = 1;	// Wait the reception of new data in the next state
					else
						rxState = 3;	// Check if the packet has been completely received
					
					break;
			
			
				case 3:		
					// Check if the packet has been fully received
					if(bytesReceived - headerIndex >= sizeof(RET_PACKET) - 1)
						packetReceived = true;
					else
						packetReceived = false;
					
					// Decide which is the next state
					if(packetReceived == false)
						rxState = 1;	// Wait the reception of new data in the next state
				
					break;
			}
		}
		
		// Extract the data packet
		for(k = 0; k < sizeof(RET_PACKET); k++)
			tempBuffer[k] = buffer[k + headerIndex];
		
		servoDataPacket = (RET_PACKET*)tempBuffer;
		if(strncmp((char*)servoDataPacket->header, header, 3) == 0)
			packetCorrect = 1;
		else
			cout << "WARNING: [in servoDataThread] packet header not recognized" << endl;
		
		// Send the control references to the servos if packet is correct
		if(packetCorrect == 1)
		{		
			gettimeofday(&t1, NULL);
			t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec);
			
			servoPosition = 0.01*servoDataPacket->servoPosition;
			servoSpeed = 0.1*servoDataPacket->servoSpeed;
			motorCurrent = 0.01*servoDataPacket->motorCurrent;
			motorTorque = 1.0*servoDataPacket->motorTorque;
			
			threadArgs->mhdActuatorPosition = servoPosition;
			threadArgs->mhdActuatorSpeed = servoSpeed;
			threadArgs->mhdActuatorTorque = motorTorque;
			
			printf(">> Servo position: %.2f [deg]\n", servoPosition);
			// printf(">> Servo speed: %.2f [deg/s]\n", servoSpeed);
			// printf(">> Motor torque: %.2f [mNm]\n", motorTorque);
			
			servoPosition_1 = servoPosition;
			t_1 = t;
		}
		else
			cout << "Packet error" << endl;
		
		// Get time stamp at the end of the loop
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
	}
	
		
	/******************************** THREAD LOOP END ********************************/
		
	cout << "MHD Servo Data thread terminated" << endl;


	return 0;
}



#endif 


