/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ThreadsControl.h
 */
 
 
 
// Standard library
#include <iostream>
#include <fstream>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>


// Specific library
#include "../ArmController/ArmController.h"
#include "../Constants/Constants.h"
#include "../DataLog/DataLog.h"


// Constant definition
#define NUM_WAY_POINTS 100


// Thread function declaration
static void * gcsThreadFunction(void * args);
static void * dataLogThreadFunction(void * args);
static void * keyboardThreadFunction(void * args);


// Namespaces
using namespace std;



static void * gcsThreadFunction(void * args)
{
	THREAD_ARGS * threadArg = (THREAD_ARGS*)args;
	GCS_PACKET * gcsPacket = NULL;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	bool endFlag = false;
	struct timeval t0keepAlive;
	struct timeval t1keepAlive;
	double keepAliveTimer = 0;
	bool resetKeepAliveTimer = true;
	int error = 0;
	
	
	cout << "GCS interface thread started" << endl;

	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		error = 1;
		cout << endl << "ERROR: [in gcsThreadFunction] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(threadArg->udpListeningPort);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in gcsThreadFunction] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && threadArg->endFlag == false)
	{
		// Get initial time stamp for the keep alive message
		if(resetKeepAliveTimer == true)
		{
			resetKeepAliveTimer = false;
			gettimeofday(&t0keepAlive, NULL);
		}
		
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if(dataReceived > 0)
		{
			// Check if message is correct
			if(buffer[0] == 'G' && buffer[1] == 'C' && buffer[2] == 'S')
			{
				// Reset keep alive timer flag
				resetKeepAliveTimer = true;
				
				// Get the code sent by the GCS
				gcsPacket = (GCS_PACKET*)buffer;
				
				if(gcsPacket->code != 0)
				{
					threadArg->gcsCodeReceivedFlag = true;
					threadArg->gcsCode = gcsPacket->code;
				}
				if(gcsPacket->code < 0)
					threadArg->endFlag = true;
			}
		}
		else
		{
			// Wait 10 ms
			usleep(10000);
		}
		
		// Update keep alive timer
		gettimeofday(&t1keepAlive, NULL);
		keepAliveTimer = (t1keepAlive.tv_sec - t0keepAlive.tv_sec) + 1e-6*(t1keepAlive.tv_usec - t0keepAlive.tv_usec);
		if(keepAliveTimer > 5)
		{
			threadArg->keepAliveWarning = 1;
			cout << "WARNING: [in gcsThreadFunction] Keep Alive Timer exceeded! Retract arms!" << endl;
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);
	
	cout << "GCS interface thread terminated" << endl;
	
	
	return 0;
}



static void * teleoperationThreadFunction(void * args)
{
	THREAD_ARGS * threadArg = (THREAD_ARGS*)args;
	TELEOP_PACKET * teleopPacket = NULL;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int error = 0;
	
	
	cout << "Teleoperation interface thread started" << endl;

	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		error = 1;
		cout << endl << "ERROR: [in teleoperationThreadFunction] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(22001);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in teleoperationThreadFunction] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && threadArg->endFlag == false)
	{	
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if (dataReceived > 0)
		{
			// Check if message is correct
			if(buffer[0] == 'T' && buffer[1] == 'O' && buffer[2] == 'P')
			{	
				// Get the code sent by the GCS
				teleopPacket = (TELEOP_PACKET*)buffer;
				
				threadArg->teleoperatedPosition[0] = teleopPacket->x;
				threadArg->teleoperatedPosition[1] = teleopPacket->y;
				threadArg->teleoperatedPosition[2] = teleopPacket->z;
				threadArg->teleoperatedOrientation[0] = teleopPacket->roll;
				threadArg->teleoperatedOrientation[1] = teleopPacket->pitch;
				threadArg->teleoperatedOrientation[2] = teleopPacket->yaw;
				threadArg->leftButton3DConnexion = (uint8_t)teleopPacket->buttons[0];
				threadArg->rightButton3DConnexion = (uint8_t)teleopPacket->buttons[1];
				// printf("Teleoperation reference: {%.3lf, %.3lf, %.3lf}\n", threadArg->teleoperatedPosition[0], threadArg->teleoperatedPosition[1], threadArg->teleoperatedPosition[2]);
				// printf("Buttons: {%d, %d}\n", teleopPacket->leftButton, teleopPacket->rightButton);
			}
		}
		else
		{
			// Wait 1 ms
			usleep(1000);
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);
	
	cout << "Teleoperation interface thread terminated" << endl;
	
	
	return 0;
}



static void * dataLogThreadFunction(void * args)
{
	THREAD_ARGS * threadArg = (THREAD_ARGS*)args;
	DataLog * dataLog = new DataLog();
	struct timeval tini;
	struct timeval tend;
	double t = 0;
	int error = 0;
	
	
	cout << "Data log thread started" << endl;

	// Create the log files
	dataLog->createLogFiles();

	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && threadArg->endFlag == false)
	{
		// Get time stamp at the begining of the control loop
		gettimeofday(&tini, NULL);
		
		
		// Write data on files
		dataLog->writeArmData(LEFT_ARM_ID, threadArg->leftArmController, threadArg->leftArmKinematics);
		dataLog->writeArmData(RIGHT_ARM_ID, threadArg->rightArmController, threadArg->rightArmKinematics);
		dataLog->writeVisionData(threadArg->leftArmMarkerPosition, threadArg->rightArmMarkerPosition);
		dataLog->writeVisionDeflectionData(threadArg->visionDeflection.leftMarkerPosition, threadArg->visionDeflection.leftMarkerVelocity, threadArg->visionDeflection.rightMarkerPosition, threadArg->visionDeflection.rightMarkerVelocity);
		dataLog->writeGCSData(threadArg->gcsCode, threadArg->teleoperatedPosition, threadArg->teleoperatedOrientation, threadArg->leftButton3DConnexion, threadArg->rightButton3DConnexion);

		
		// Get time stamp at the end of the control loop
		gettimeofday(&tend, NULL);
		t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
		
		// Wait time
		if(t > DATA_LOG_PERIOD)
			cout << "WARNING: [in dataLogThreadFuction]: updat period exceeded." << endl;
		else
			usleep((useconds_t)(1e6*(DATA_LOG_PERIOD - t)));
	}
	
	/******************************** THREAD LOOP END ********************************/

	// Close the log files
	dataLog->closeLogFiles();
			
	cout << "Data log thread terminated" << endl;
	

	return 0;
}



static void * keyboardThreadFunction(void * args)
{
	THREAD_ARGS * threadArg = (THREAD_ARGS*)args;
	GCS_PACKET * gcsPacket = NULL;
	int key = 0;
	int error = 0;
	
	
	cout << "Keyboard thread started" << endl;

	/******************************** THREAD LOOP START ********************************/
	
	while(key != -1 && error == 0 && threadArg->endFlag == false)
	{
		scanf("%d", &key);
		threadArg->key = key;
		usleep(100000);
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	cout << "Keyboard thread terminated" << endl;
	
	
	return 0;
}
