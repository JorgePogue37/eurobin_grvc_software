/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: TaskManager.cpp
 */

#include "TaskManager.h"


void TaskManager::receiveGCSCommand()
{
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
		cout << endl << "ERROR: [in receiveGCSCommand] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(UDP_PORT_GCS);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in receiveGCSCommand] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && endThreadsSignal == false)
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
					this->gcsCodeReceivedFlag = 1;
					this->gcsCode = gcsPacket->code;
				}
				if(gcsPacket->code < 0)
					this->endThreadsSignal = true;
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
			cout << "WARNING: [in receiveGCSCommand] Keep Alive Timer exceeded! Retract arms!" << endl;
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);
	
	cout << "GCS interface thread terminated" << endl;
}


/*
 * Thread for receiving the 3DConnexion joystick references
 */
void TaskManager::receive3DConnexionMouseData()
{
	DATA_PACKET_3DCONNEXION * teleopPacket;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int error = 0;
	
	
	cout << "3DConnexion data reception thread started" << endl;

	// Set to zero the data packet
	joystickTeleopData.x = 0;
	joystickTeleopData.y = 0;
	joystickTeleopData.z = 0;
	joystickTeleopData.roll = 0;
	joystickTeleopData.pitch = 0;
	joystickTeleopData.yaw = 0;
	joystickTeleopData.buttons[0] = 0;
	joystickTeleopData.buttons[1] = 0;

	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		error = 1;
		cout << endl << "ERROR: [in TaskManager::receive3DConnexionMouseData] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(UDP_PORT_3DCONNEXION);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in TaskManager::receive3DConnexionMouseData] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && endThreadsSignal == 0)
	{	
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if (dataReceived > 0)
		{
			// Check if message is correct
			if(buffer[0] == 'T' && buffer[1] == 'O' && buffer[2] == 'P')
			{	
				// Get the code sent by the GCS
				teleopPacket = (DATA_PACKET_3DCONNEXION*)buffer;
				
				this->joystickTeleopData.x = teleopPacket->x;	
				this->joystickTeleopData.y = teleopPacket->y;
				this->joystickTeleopData.z = teleopPacket->z;
				this->joystickTeleopData.roll = teleopPacket->roll;	
				this->joystickTeleopData.pitch = teleopPacket->pitch;
				this->joystickTeleopData.yaw = teleopPacket->yaw;
				this->joystickTeleopData.buttons[0] = (uint8_t)teleopPacket->buttons[0];
				this->joystickTeleopData.buttons[1] = (uint8_t)teleopPacket->buttons[1];
				// printf("3DConnexion XYZ: {%.3lf, %.3lf, %.3lf}\n", joystickTeleopData.x, joystickTeleopData.y, joystickTeleopData.z);
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
	
	cout << "3DConnexion data reception thread terminated" << endl;
}


/*
 * Send signal to signal manager through UDP interface
 */
int TaskManager::sendSignalUDPSocket(uint8_t _moduleID, uint8_t _signal, uint8_t _signal2)
{
	DATA_PACKET_STATE_SIGNAL signalDataPacket;
	int bytesSent = 0;
	int error = 0;
	
	
	signalDataPacket.header[0] = 'S';
	signalDataPacket.header[1] = 'I';
	signalDataPacket.header[2] = 'G';
	signalDataPacket.moduleID = _moduleID;
	signalDataPacket.signal = _signal;
	signalDataPacket.signal2 = _signal2;
	
	bytesSent = sendto(socketSignalPublisher, (char*)&signalDataPacket, sizeof(DATA_PACKET_STATE_SIGNAL), 0, (struct sockaddr*)&addrHost, sizeof(struct sockaddr));
	if(bytesSent < 0)
	{
		error = 1;
		cout << "ERROR [TaskManager::sendSignalUDPSocket]: could not send data through socket." << endl;
	}
	else if(bytesSent != sizeof(DATA_PACKET_STATE_SIGNAL))
	{
		error = 1;
		cout << "ERROR [TaskManager::sendSignalUDPSocket]: could not send complete data packet." << endl;
	}
	
	
	return error;
}

	
/*
 * Receive data from visual sensor
 */
void TaskManager::receiveVisionModuleData()
{
	DATA_PACKET_VISION_MODULE * visionModuleDataPacket;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int k = 0;
	int error = 0;
	
	float R0c[3][3] = {{0.9397, 0.0, 0.342}, {0.0, 1.0, 0.0}, {-0.342, 0.0, 0.9397}};
	const float p0c[3] = {-0.3, 0.0, -0.1};	// Camera frame {C} relative to dual arm frame {0}
	float pcL[3] = {0.0, 0.0, 0.0};			// Left point in camera frame
	float pcR[3] = {0.0, 0.0, 0.0};			// Right point in camera frame
	float pL[3] = {0.0, 0.0, 0.0};
	float pR[3] = {0.0, 0.0, 0.0};
	float vL[3] = {0.0, 0.0, 0.0};
	float vR[3] = {0.0, 0.0, 0.0};
	int i = 0;
	int j = 0;
	
	
	
	cout << "Vision data reception thread started" << endl;

	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		error = 1;
		cout << endl << "ERROR: [in TaskManager::receiveVisualSensorData] could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(UDP_PORT_VISION_MODULE);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		error = 1; 
		cout << endl << "ERROR: [in TaskManager::receiveVisualSensorData] could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	
	while(error == 0 && endThreadsSignal == 0)
	{
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if(dataReceived == sizeof(DATA_PACKET_VISION_MODULE))
		{
			// Check if message is correct
			if(buffer[0] == 'G' && buffer[1] == 'R' && buffer[2] == 'P')
			{	
				// Get the code sent by the GCS
				visionModuleDataPacket = (DATA_PACKET_VISION_MODULE*)buffer;
				
				// Get the left/right points in the camera frame
				for(k = 0; k < 3; k++)
				{
					pcL[k] = visionModuleDataPacket->pL[k];
					pcR[k] = visionModuleDataPacket->pR[k];
				}
				
				// Multiply R0c*pc
				for(i = 0; i < 3; i++)
				{
					vL[i] = 0;
					vR[i] = 0;
					for(j = 0; j < 3; j++)
					{
						vL[i] += R0c[i][j]*pcL[j];
						vR[i] += R0c[i][j]*pcR[j];
					}
				}
				
				// Obtain the points in the dual arm frame
				for(k = 0; k < 3; k++)
				{
					this->visionModuleData.pL[k] = vL[k] + p0c[k];
					this->visionModuleData.pR[k] = vR[k] + p0c[k];
				}
				this->visionModuleData.validL = 1;
				this->visionModuleData.validR = 1;
				
				// printf("Vision data PL: {%.3f, %.3f, %.3f}\n", visionModuleData.pL[0], visionModuleData.pL[1], visionModuleData.pL[2]);
				// printf("Vision data PR: {%.3f, %.3f, %.3f}\n", visionModuleData.pR[0], visionModuleData.pR[1], visionModuleData.pR[2]);
				// printf("----------\n");
			}
		}
		else
		{
			// Wait 10 ms
			usleep(10000);
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);
	
	cout << "Vision data reception thread terminated" << endl;
}
	
		
/*
 * Take control references from external source
 */
void TaskManager::externalJointReferenceControl()
{
}


