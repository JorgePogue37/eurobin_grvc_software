/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ThreadInterface.h
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
#include <netdb.h>


// Specific library
#include "../ArmController/ArmController.h"
#include "../Constants/Constants.h"
#include "../DataLog/DataLog.h"


// Constant definition


// Thread function declaration
static void * armsControlReferencesThreadFunction(void * args);
static void * armsStatePublisherThreadFunction(void * args);


/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct
{
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

// Namespaces
using namespace std;



/*
 * This thread waits the reception of control references sent through an UDP socket
 */
static void * armsControlReferencesThreadFunction(void * args)
{
	THREAD_ARGS * threadArgs = (THREAD_ARGS*)args;
	ARMS_CONTROL_REFERENCES_DATA_PACKET * armsControlRefDataPacket = NULL;
	struct sockaddr_in addr;
	struct sockaddr_in addrVS;
	socklen_t addrLength;
	char buffer[1024];
	int dataReceived;
	int socketReceiver;
	int k = 0;
	struct timeval tini;
	struct timeval tend;
	double delta_t = 0;
	int error = 0;
	
	
	cout << "Control references thread started" << endl;
	
	// Open the socket in stream mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in armsControlReferencesThreadFunction] could not open socket." << endl;
	}
	else
	{
		// Set listenning address and port for server
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		// addr.sin_port = htons(CONTROL_REFERENCES_UDP_PORT);
		addr.sin_port = htons(22008);

		// Associates the address to the socket
		if(bind(socketReceiver, (struct sockaddr*)&addr, sizeof(struct sockaddr)) < 0)
		{
			error = 1;
			cout << endl << "ERROR: [in armsControlReferencesThreadFunction] could not associate address to socket." << endl;
		}
	}
	
	// Initializes the reference
	for(k = 0; k < 4; k++)
	{
		threadArgs->armsControlRefs.leftArmJointPositionRef[k] = 0;
		threadArgs->armsControlRefs.rightArmJointPositionRef[k] = 0;
	}
	for(k = 0; k < 3; k++)
	{
		threadArgs->armsControlRefs.leftArmCartesianPositionRef[k] = 0;
		threadArgs->armsControlRefs.rightArmCartesianPositionRef[k] = 0;
	}
	
	/*********************** Reception loop ***********************/
	while(error == 0 && threadArgs->endFlag == false)
	{
		// Get time stamp at the begining of the loop
		gettimeofday(&tini, NULL);
		
		dataReceived = recvfrom(socketReceiver, buffer, 1024, 0, (struct sockaddr*)&addrVS, &addrLength);
		if(dataReceived == sizeof(ARMS_CONTROL_REFERENCES_DATA_PACKET))
		{
			// cout << "Arms external reference data received" << endl;
			armsControlRefDataPacket = (ARMS_CONTROL_REFERENCES_DATA_PACKET*)buffer;
			memcpy(&(threadArgs->armsControlRefs), armsControlRefDataPacket, sizeof(ARMS_CONTROL_REFERENCES_DATA_PACKET));
			
			threadArgs->controlReferencesDataReceivedFlag = 1;
			
			double timeStamp = threadArgs->armsControlRefs.timeStamp;
			double q1Lref = 57.296*threadArgs->armsControlRefs.leftArmJointPositionRef[0];
			double q2Lref = 57.296*threadArgs->armsControlRefs.leftArmJointPositionRef[1];
			double q3Lref = 57.296*threadArgs->armsControlRefs.leftArmJointPositionRef[2];
			double q4Lref = 57.296*threadArgs->armsControlRefs.leftArmJointPositionRef[3];
			double q1Rref = 57.296*threadArgs->armsControlRefs.rightArmJointPositionRef[0];
			double q2Rref = 57.296*threadArgs->armsControlRefs.rightArmJointPositionRef[1];
			double q3Rref = 57.296*threadArgs->armsControlRefs.rightArmJointPositionRef[2];
			double q4Rref = 57.296*threadArgs->armsControlRefs.rightArmJointPositionRef[3];
			double xLref = threadArgs->armsControlRefs.leftArmCartesianPositionRef[0];
			double yLref = threadArgs->armsControlRefs.leftArmCartesianPositionRef[1];
			double zLref = threadArgs->armsControlRefs.leftArmCartesianPositionRef[2];
			double xRref = threadArgs->armsControlRefs.rightArmCartesianPositionRef[0];
			double yRref = threadArgs->armsControlRefs.rightArmCartesianPositionRef[1];
			double zRref = threadArgs->armsControlRefs.rightArmCartesianPositionRef[2];			
			// printf("SUBSCRIBER - t = %.2lf Left arm Joint reference: <%.0lf, %.0lf, %.0lf, %.0lf> [deg]\n", timeStamp, q1Lref, q2Lref, q3Lref, q4Lref);
			// printf("SUBSCRIBER - t = %.2lf Right arm Joint reference: <%.0lf, %.0lf, %.0lf, %.0lf> [deg]\n", timeStamp, q1Rref, q2Rref, q3Rref, q4Rref);
			// printf("SUBSCRIBER - t = %.2lf Left arm Joint/Cartesian reference: <%.0lf, %.0lf, %.0lf, %.0lf> [deg], {%.3lf, %.3lf, %.3lf} [m]", timeStamp, q1Lref, q2Lref, q3Lref, q4Lref, xLref, yLref, zLref);
			// printf("SUBSCRIBER - t = %.2lf Right arm Joint/Cartesian reference: <%.0lf, %.0lf, %.0lf, %.0lf> [deg], {%.3lf, %.3lf, %.3lf} [m]", timeStamp, q1Rref, q2Rref, q3Rref, q4Rref, xRref, yRref, zRref);
			// printf("XYZ_L_ref = {%.1lf, %.1lf, %.1lf} [cm]\n", 100*xLref, 100*yLref, 100*zLref);
			// printf("XYZ_R_ref = {%.1lf, %.1lf, %.1lf} [cm]\n", 100*xRref, 100*yRref, 100*zRref);
			// cout << "______________" << endl << endl;
		}
		else
			printf("Packet size is not correct. %ld bytes expected - %d received\n", sizeof(ARMS_CONTROL_REFERENCES_DATA_PACKET), dataReceived);
			
		// Get time stamp at the end of the loop and estimate data rate
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
		// printf("Data rate: %.1f\n", 1/delta_t);
	}
	
	
	return 0;
}



static void * armsStatePublisherThreadFunction(void * args)
{
	THREAD_ARGS * threadArgs = (THREAD_ARGS*)args;
	ARMS_STATE_PUBLISHER_DATA_PACKET armsStatePublisherDataPacket;
	struct sockaddr_in addrHost;
    struct hostent * host;
	int socketPublisher = -1;
	struct timeval tini;
	struct timeval tend;
	double t = 0;
	int error = 0;
	
	
	cout << "Arms state publisher thread started" << endl;


	// Open the socket in datagram mode
	socketPublisher = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socketPublisher < 0)
    {
    	cout << endl << "ERROR: [in armsStatePublisherThreadFunction(...)] could not open socket." << endl;
    	error = 1;
   	}
		
   	host = gethostbyname(STATE_PUBLISHER_IP_ADDRESS);
    if(host == NULL)
	{
	    cout << "ERROR: [in armsStatePublisherThreadFunction(...)] could not get host by name" << endl;
	    error = 1;
	}
	
	// Set the address of the host
	if(error == 0)
	{
		bzero((char*)&addrHost, sizeof(struct sockaddr_in));
		addrHost.sin_family = AF_INET;
		bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
		addrHost.sin_port = htons(STATE_PUBLISHER_UDP_PORT);
	}
	else
		close(socketPublisher);


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && threadArgs->endFlag == false)
	{
		// Get initial time stamp
		gettimeofday(&tini, NULL);
		
		// Build the packet
		armsStatePublisherDataPacket.leftArmJointValues[0] = 0.01745*threadArgs->leftArmController->shoulderPitchServoState->position;
		armsStatePublisherDataPacket.leftArmJointValues[1] = 0.01745*threadArgs->leftArmController->shoulderRollServoState->position;
		armsStatePublisherDataPacket.leftArmJointValues[2] = 0.01745*threadArgs->leftArmController->shoulderYawServoState->position;
		armsStatePublisherDataPacket.leftArmJointValues[3] = 0.01745*threadArgs->leftArmController->elbowPitchServoState->position;
		armsStatePublisherDataPacket.rightArmJointValues[0] = 0.01745*threadArgs->rightArmController->shoulderPitchServoState->position;
		armsStatePublisherDataPacket.rightArmJointValues[1] = 0.01745*threadArgs->rightArmController->shoulderRollServoState->position;
		armsStatePublisherDataPacket.rightArmJointValues[2] = 0.01745*threadArgs->rightArmController->shoulderYawServoState->position;
		armsStatePublisherDataPacket.rightArmJointValues[3] = 0.01745*threadArgs->rightArmController->elbowPitchServoState->position;
		armsStatePublisherDataPacket.leftTCPCartesianPosition[0] = threadArgs->leftArmController->endEffectorPosition[0];
		armsStatePublisherDataPacket.leftTCPCartesianPosition[1] = threadArgs->leftArmController->endEffectorPosition[1];
		armsStatePublisherDataPacket.leftTCPCartesianPosition[2] = threadArgs->leftArmController->endEffectorPosition[2];
		armsStatePublisherDataPacket.rightTCPCartesianPosition[0] = threadArgs->rightArmController->endEffectorPosition[0];
		armsStatePublisherDataPacket.rightTCPCartesianPosition[1] = threadArgs->rightArmController->endEffectorPosition[1];
		armsStatePublisherDataPacket.rightTCPCartesianPosition[2] = threadArgs->rightArmController->endEffectorPosition[2];
		
		// printf("PUBLISHER: Left arm joints: <%.1lf, %.1lf, %.1lf, %.1lf> [deg]\n", 57.296*armsStatePublisherDataPacket.leftArmJointValues[0], 57.296*armsStatePublisherDataPacket.leftArmJointValues[1], 57.296*armsStatePublisherDataPacket.leftArmJointValues[2], 57.296*armsStatePublisherDataPacket.leftArmJointValues[3]);
		// printf("PUBLISHER: Right arm joints: <%.1lf, %.1lf, %.1lf, %.1lf> [deg]\n", 57.296*armsStatePublisherDataPacket.rightArmJointValues[0], 57.296*armsStatePublisherDataPacket.rightArmJointValues[1], 57.296*armsStatePublisherDataPacket.rightArmJointValues[2], 57.296*armsStatePublisherDataPacket.rightArmJointValues[3]);
		// cout << endl;
		
		// Send the packet
		if(sendto(socketPublisher, (char*)&armsStatePublisherDataPacket, sizeof(ARMS_STATE_PUBLISHER_DATA_PACKET), 0, (struct sockaddr*)&addrHost, sizeof(struct sockaddr)) < 0)
		{
			error = 1;
			cout << "ERROR: [in referenceSenderThreadFunction(...)] could not send data." << endl;
		}
		
		// Compute the elapsed time
		gettimeofday(&tend, NULL);
		t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
		
		if(t > ARMS_STATE_PUBLISHER_PERIOD)
			cout << "WARNING: [in referenceSenderThreadFunction(...)] update period exceeded" << endl;
		else
			usleep((useconds_t)(1e6*(ARMS_STATE_PUBLISHER_PERIOD - t)));
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketPublisher);
	
	cout << "Arms state publisher thread terminated" << endl;
	
	
	return 0;
}





