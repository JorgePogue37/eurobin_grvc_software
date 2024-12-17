/*
 * Copyright (c) 2017 Alejandro Suarez Fernandez-Miranda
 *
 * This source code is part of the PhD thesis "Dexterous Aerial Manipulation"
 * and part of the AEROARMS project.
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: ArmController.cpp
 */


#include "ArmController.h"



/*
 * Constructor
 * */
ArmController::ArmController(uint8_t _armID)
{
	// Set the ID of the arm
	armID = _armID;
	
	// Set the thread end signal to zero
	endThreadSignal = 0;
}


/*
 * Destructor
 * */
ArmController::~ArmController()
{
	// Clear vectors
	armServos.clear();
	servosID.clear();
	servosModel.clear();
	servosSignCorrection.clear();
	servosOffsetAngle.clear();
	jointsLimitMax.clear();
	jointsLimitMin.clear();
}

	
/*
 * Initiate arm controller: set the ID's of the servos, sign and offset corrections, joint limits, and set torque ON.
 */
int ArmController::init(int _numArmJoints, uint8_t *_servoIDs, int * _servoModel, int * _signCorrections, float * _offsetCorrections, float * _maxJointLimits, float * _minJointLimits, const string &_uartDeviceName)
{
	int k = 0;
	int error = 0;
	
	
	// Set the number of joints of the arms
	numArmJoints = _numArmJoints;

	// Create the USB-UART device interface for the arm
	usbDevice = this->createSerialInterface(_uartDeviceName.c_str(), B115200);
	if(usbDevice <= 0)
	{
		error = 1;
		printf("ERROR [in ArmController::init]: could not create UART interface for arm %d\n", this->armID);
	}
	else
	{
		// Init arm kinematics
		armKinematics = new Kinematics(armID, 0.0, 0.0, 0.0);
		
		// Create the vector of servos and assign parameters
		for(k = 0; k < numArmJoints; k++)
		{
			armServos.push_back(ServoState(_servoIDs[k], _servoModel[k], _signCorrections[k], _offsetCorrections[k]));
			servosID.push_back(_servoIDs[k]);
			servosModel.push_back(_servoModel[k]);
			servosSignCorrection.push_back(_signCorrections[k]);
			servosOffsetAngle.push_back(_offsetCorrections[k]);
			jointsLimitMax.push_back(_maxJointLimits[k]);
			jointsLimitMin.push_back(_minJointLimits[k]);
		}
		
		// Enable torque control on all the servos
		for(k = 0; k < numArmJoints; k++)
		{
			this->servoTorqueControl(_servoIDs[k], 1);
			usleep(10000);
		}
		
		// Create arm thread
		armThread = thread(&ArmController::updateArmServosState, this);
		armThread.detach();
	}
	
	
	return error;
}


/*
 * Terminate the arm servos data thread
 */
void ArmController::endServoThread()
{
	printf("Terminating arm-%d servos thread...", armID);
	endThreadSignal = 1;
	usleep(10000);
	printf("Done\n");
}


/*
 * Thread function for updating arm servos state
 */
void ArmController::updateArmServosState()
{
	struct timeval tini;
	struct timeval tend;
	double t = 0;
	double elapsedTime = 0;
	ssize_t bytesRead = 0;
	unsigned int k = 0;
	unsigned int headerIndex = 0;
	ssize_t bytesReceived = 0;
	uint8_t bufferRx[RX_BUFFER_LENGTH];
	uint8_t buffer[RX_BUFFER_LENGTH];
	uint8_t bufferAux[RX_BUFFER_LENGTH];
	uint8_t tempBuffer[RX_BUFFER_LENGTH];
	uint8_t rxState = 1;
	uint8_t indexServo = 0;
	int servoDataPacketLength = 0;
	bool headerFound = false;
	bool packetReceived = false;
	bool bufferOverflow = false;
	
	int error = 0;
	

	printf("Arm-%d servos data thread started\n", this->armID);
	
	// Init data acquisition loop
	while(error == 0 && endThreadSignal == 0)
	{
		// Get current time stamp
		gettimeofday(&tini, NULL);
		
		// Send the RAM read request packet to the corresponding servo
		this->armServos[indexServo].sendStatusUpdateRequest(this->usbDevice);
		if(servosModel[indexServo] == 402 || servosModel[indexServo] == 602)
		{
			servoDataPacketLength = SERVO_DATA_PACKET_LENGTH_0402;
		}
		else
			servoDataPacketLength = SERVO_DATA_PACKET_LENGTH_0101;
			
		/****************************** RECEPTION OF DATA FROM SERVOS INI ******************************/
			
		// Initialize variables of the state machine
		bytesReceived = 0;
		rxState = 1;
		headerFound = false;
		packetReceived = false;
				
		while(packetReceived == false && error == 0 && this->endThreadSignal == false)
		{
			switch (rxState)
			{
				case 1:
					// Wait the reception of a new data packet
					bytesRead = read(usbDevice, (char*)bufferRx, RX_BUFFER_LENGTH - 1);
					if(bytesRead > 0)
					{
						if(bytesReceived + bytesRead >= RX_BUFFER_LENGTH)
						{
							bufferOverflow = true;
							bytesReceived = 0;
							headerFound = false;
							packetReceived = false;
							cout << "WARNING: [in ArmController::updateArmServosState] buffer overflow." << endl;
						}
						else
						{
							// Copy the received data into the buffer
							for(k = 0; k < bytesRead; k++)
								buffer[k + bytesReceived] = bufferRx[k];
							bytesReceived += bytesRead;
						}
					}
					else
						usleep(1000);	// Wait 1 ms
		
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
					// Look for packet header (0x255 0x255)
					headerFound = false;
					for(k = 0; k < bytesReceived - 2 && headerFound == false; k++)
					{
						if(buffer[k] == 255 && buffer[k+1] == 255)
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
					if(bytesReceived - headerIndex >= servoDataPacketLength)
						packetReceived = true;
					else
						packetReceived = false;
							
					// Decide which is the next state
					if(packetReceived == false)
						rxState = 1;	// Wait the reception of new data in the next state
						
					break;
			}
		}
			
		/****************************** RECEPTION OF DATA FROM SERVOS END ******************************/
		
		// Extract the data packet
		for(k = 0; k < servoDataPacketLength; k++)
			tempBuffer[k] = buffer[k + headerIndex];
		
		// Update the state of the corresponding servo
		this->updateServoState((char*)tempBuffer, servoDataPacketLength, 402);
		
		// Update the index of the next servo to read
		indexServo++;
		if(indexServo >= this->armServos.size())
			indexServo = 0;
			
		// Get time stamp at the end of the loop
		gettimeofday(&tend, NULL);
		t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini.tv_usec);
			
		// Wait to complete a RAM read period
		if(t > SERVO_RAM_READ_INTERVAL)
			// cout << "WARNING: [in ArmController::updateArmServosState] servo RAM read period is exceeded." << endl;
			;
		else
			usleep((useconds_t)(1e6*(SERVO_RAM_READ_INTERVAL - t)));
	}
	
	printf("Arm-%d servos data thread terminated\n", this->armID);
}
	
	
/*
 * Update servo state from the received data packet. The ID of the corresponding servo is determined from the data packet.
 * The state of the servo is referred to its shaft frame (no sign or offset conversion is applied).
 */
int ArmController::updateServoState(char * dataPacket, int bytesReceived, int servoModel)
{
	uint8_t servoID = 0;
	int servoFound = 0;
	int servoDataPacketLength = 0;
	int k = 0;
	int error = 0;
	
	
	// Determien the length of the input packet according to the servo model
	if(servoModel == 101 || servoModel == 201)
		servoDataPacketLength = SERVO_DATA_PACKET_LENGTH_0101;
	if(servoModel == 402 || servoModel == 602)
		servoDataPacketLength = SERVO_DATA_PACKET_LENGTH_0402;
	
	// Check if the size of the received packet is correct
	if(bytesReceived < servoDataPacketLength)
	{
		error = 1;
		cout << "WARNING: [in ArmController::updateServoState] packet size is not correct" << endl;
		cout << "Bytes received: " << bytesReceived << endl;
	}
	else
	{
		// Extract the fields onf the data packet
		uint8_t h1 = (uint8_t)dataPacket[0];
		uint8_t h2 = (uint8_t)dataPacket[1];
		uint8_t h3 = (uint8_t)dataPacket[4];
		if(h1 != 255 || h2 != 255 || h3 != 68)
		{
			error = 1;
			cout << "WARNING: [in ArmController::updateServoState] packet header not recognized" << endl;
		}
		else
		{
			// Get servo ID
			servoID = (uint8_t)dataPacket[3];
			
			// Update the state of the servo. The sign and offset corrections are applied within the ServoState object.
			for(k = 0; k < numArmJoints && servoFound == 0; k++)
			{
				if(servoID == servosID[k])
				{
					servoFound = 1;
					armServos[k].updateServoState(dataPacket);
				}
			}
			
			if(servoFound == 0)
			{
				printf("WARNING: [in ArmController::updateServoState] servo ID %d not recognized on arm %d\n", servoID, armID);
				printf("IDs of the active servos for arm %d are: ", armID);
				for(k = 0; k < numArmJoints; k++)
					printf("[%d] ", servosID[k]);
				printf("\n");
			}
		}
	}
	
	
	return error;
}


/*
 * Update the reference Cartesian position of the end effector
 */
void ArmController::updateEndEffectorRefPosition(double xref, double yref, double zref)
{
	tcpPositionReference[0] = xref;
	tcpPositionReference[1] = yref;
	tcpPositionReference[2] = zref;
}


/*
 * Enable/disable torque control of the arm.
 */
void ArmController::armTorqueControl(uint8_t controlMode)
{
	for(int k = 0; k < numArmJoints; k++)
	{
		this->servoTorqueControl(servosID[k], controlMode);
		usleep(2000);
	}
}

	
/*
 * Enable/disable torque control of the specified servo
 */
void ArmController::servoTorqueControl(uint8_t servoID, uint8_t controlMode)
{
	uint8_t packet[16];
	uint8_t packetLength = 0;
	
	
	// Set the fields of the packet
	packet[0] = 0xFF;		// Header
	packet[1] = 0xFF;
	packet[2] = 0x0A;		// Packet length
	packet[3] = servoID;	// Servo ID
	packet[4] = 0x03;		// RAM write
	packet[7] = 0x34;		// Register
	packet[8] = 0x01;		// Bytes to write
	switch (controlMode)
	{
		case 0:
			packet[9] = 0x00;	// Torque free
			break;
		case 1:
			packet[9] = 0x60;	// Torque ON
			break;
		case 2:
			packet[9] = 0x40;	// Break ON
			break;
		default:
			packet[9] = 0x00;	// Torque free
			break;
	}
	
	// Compute the checksum
	computeChecksum((char*)packet, packet[2]);
	
	// Send the packet
	sendData((char*)packet, packet[2], this->usbDevice);
}
	
/*
 * Move the joints of the arm to the desired position in the specified time.
 */
int ArmController::moveJoints(float * q_ref, float playTime)
{
	int k = 0;
	int error = 0;
	
	
	for(k = 0; k < this->numArmJoints; k++)
	{
		error += this->moveJoint(this->servosID[k], q_ref[k], playTime);
		usleep(4000);
	}
	
	
	return error;
}

	
/*
 * Return current joint position in [rad]
 */
void ArmController::getJointsPosition(float * q)
{
	int k = 0;
	
	for(k = 0; k < this->numArmJoints; k++)
		q[k] = armServos[k].position;
}

	
/*
 * Return current Cartesian position in [m]
 */
void ArmController::getCartesianPosition(float * pos)
{
	double p_xyz[3] = {0.0, 0.0, 0.0};
	double q1234[4] = {0.0, 0.0, 0.0, 0.0};
	
	
	q1234[0] = 0.01745*this->armServos[0].position;
	q1234[3] = 0.01745*this->armServos[1].position;
	this->armKinematics->directKinematics(q1234[0], q1234[1], q1234[2], q1234[3], &p_xyz[0], &p_xyz[1], &p_xyz[2]);
	
	pos[0] = p_xyz[0];
	pos[1] = p_xyz[1];
	pos[2] = p_xyz[2];
	
	// printf("Arm-%d position: {%.1f, %.1f, %.1f} [cm]\n", this->armID, 100*pos[0], 100*pos[1], 100*pos[2]);
}
	
	
/*
 * Move arms to desired Cartesian position
 * */
int ArmController::goToCartesianPosition(float playTime, float * pos_ref)
{
	int error = 0;
	
	
	return error;
}
	
	
/*
 * Move arms to desired Cartesian position, specifying the q2 joint angles (redundant joints)
 * */
int ArmController::goToCartesianPosition_q2(float playTime, float * pos_ref, float q2)
{
	double q1234_ref[4] = {0.0, 0.0, 0.0, 0.0};
	float q_ref[4] = {0.0, 0.0};
	int error = 0;
	
	
	if(pos_ref[2] >= ARM_TCP_Z_MAX)
		pos_ref[2] = ARM_TCP_Z_MAX;
	
	error = this->armKinematics->IKSolver(&q1234_ref[0], &q1234_ref[1], &q1234_ref[2], &q1234_ref[3], pos_ref[0], pos_ref[1], pos_ref[2], q2);
	if(error == 0)
	{
		q_ref[0] = 57.296*q1234_ref[0];
		q_ref[1] = 57.296*q1234_ref[3];
		this->moveJoints(q_ref, playTime);
	}
	else
		cout << "WARNING [ArmController::goToCartesianPosition_q2]: kinematic error." << endl;
	
	
	return error;
}
	
	
/*
 * Move arms to desired Cartesian position, specifying the q2 joint angles (redundant joints)
 * */
int ArmController::goToCartesianPosition_q2_LinksLength(float playTime, float * pos_ref, float q2, float l1, float l2)
{
	int error = 0;
	
	
	return error;
}


/*
 * Move the indicated joint to the desired absolute position in the specified time. Offset and sign corrections
 * are applied, so the desired goal position is defined with respect to the arm frame.
 */
int ArmController::moveJoint(uint8_t servoID, float position, float playTime)
{
	uint8_t packet[16];
	uint8_t packetLength = 0;
	float goalPosition = 0;
	float playTimeByte = 0;
	float positionAux = 0;
	float positionCorrected = 0;
	uint16_t position2Bytes = 0;
	int k = 0;
	int servoFound = 0;
	int index = 0;
	int error = 0;
	
	
	// Angle(Degree) = 0.02778·(Position Raw Data - 16384)
	// Play time 1 = 11.2 ms

	// Set the fields of the packet
	packet[0] = 0xFF;		// Header
	packet[1] = 0xFF;
	packet[2] = 0x0C;		// Packet size
	packet[3] = servoID;	// Servo ID
	packet[4] = 0x05;		// I_JOG command
	
	/* ----------------- DATA SPECIFIC TO EACH SERVO ----------------- */
		// Apply the sign and offset corrections, and the joint limits
	for(k = 0; k < numArmJoints && servoFound == 0; k++)
	{
		if(servoID == servosID[k])
		{
			index = k;
			servoFound = 1;
			positionCorrected = servosSignCorrection[k]*(position + servosOffsetAngle[k]);
			goalPosition = limitServoRotation(servoID, positionCorrected);
		}
	}
	
	if(servoFound == 0)
	{
		printf("ERROR: [in ArmController::moveJoint] servo ID %d not recognized on arm %d.\n", servoID, armID);
		error = 1;
	}
	else
	{
		// Compute and set goal position (NOTE: different range for DRS-0402/0602 and DRS-0101/0201)
		if(servosModel[index] == 402 || servosModel[index] == 602)
			positionAux = 16384 + goalPosition / 0.02778;
		else if(servosModel[index] == 101 || servosModel[index] == 201)
			positionAux = 512 + goalPosition / 0.325;
		else
		{
			printf("ERROR: [in ArmController::moveJoint] servo ID %d model not recognized on arm %d.\n", servoID, armID);
			error = 1;
		}

		position2Bytes = (uint16_t)positionAux;
		packet[7] = (uint8_t)(0x00FF & position2Bytes);
		position2Bytes = position2Bytes >> 8;
		packet[8] = (uint8_t)(0x00FF & position2Bytes);
		
		packet[9] = (uint8_t)8;		// SET: Stop, Position/Velocity control mode, Green/Blue/Red LED, Disable VOR
		packet[10] = servoID;		// Servo ID
		
		// Compute and set play time in the range 0 - 255
		playTimeByte = playTime / 0.0112;
		if(playTimeByte < 1)
			playTimeByte = 1;
		if(playTimeByte > 255)
			playTimeByte = 255;
		packet[11] = (uint8_t)playTimeByte;
		/* --------------------------------------------------------------- */
		
		// Compute the checksum
		computeChecksum((char*)packet, packet[2]);
		
		// Send the packet
		if(error == 0)
			sendData((char*)packet, packet[2], this->usbDevice);
		else
			cout << "ERROR [in ArmController::moveJoint]: could not send data to servo." << endl;
	}
	
	return error;
}	


/*
 * Move the indicated servo to the desired absolute position in the specified time. Only the sign correction
 * is applied (no offset correction). This function should be only used for calibration and servo identification.
 */
int ArmController::moveServo(uint8_t servoID, float position, float playTime)
{
	uint8_t packet[16];
	uint8_t packetLength = 0;
	float goalPosition = 0;
	float playTimeByte = 0;
	float positionAux = 0;
	float positionCorrected;
	uint16_t position2Bytes = 0;
	int k = 0;
	int servoFound = 0;
	int index = 0;
	int error = 0;
	
	
	// Angle(Degree) = 0.02778·(Position Raw Data - 16384)
	// Play time 1 = 11.2 ms

	// Set the fields of the packet
	packet[0] = 0xFF;		// Header
	packet[1] = 0xFF;
	packet[2] = 0x0C;		// Packet size
	packet[3] = servoID;	// Servo ID
	packet[4] = 0x05;		// I_JOG command
	
	// ----------------- DATA SPECIFIC TO EACH SERVO ----------------- //
	
	// Apply the sign and offset corrections, and the joint limits
	for(k = 0; k < numArmJoints && servoFound == 0; k++)
	{
		if(servoID == servosID[k])
		{
			servoFound = 1;
			positionCorrected = servosSignCorrection[k]*position;
			goalPosition = limitServoRotation(servoID, positionCorrected);
		}
	}
	
	if(servoFound == 0)
	{
		printf("ERROR: [in ArmController::moveServo] servo ID %d not recognized on arm %d.\n", servoID, armID);
		error = 1;
	}
	else
	{
		// Compute and set goal position (NOTE: different range for DRS-0402/0602 and DRS-0101/0201)
		if(servosModel[index] == 402 || servosModel[index] == 602)
			positionAux = 16384 + goalPosition / 0.02778;
		else if(servosModel[index] == 101 || servosModel[index] == 201)
			positionAux = 512 + goalPosition / 0.325;
		else
		{
			printf("ERROR: [in ArmController::moveServo] servo ID %d model not recognized on arm %d.\n", servoID, armID);
			error = 1;
		}
	
		position2Bytes = (uint16_t)positionAux;
		packet[7] = (uint8_t)(0x00FF & position2Bytes);
		position2Bytes = position2Bytes >> 8;
		packet[8] = (uint8_t)(0x00FF & position2Bytes);
		
		packet[9] = (uint8_t)8;		// SET: Stop, Position/Velocity control mode, Green/Blue/Red LED, Disable VOR
		packet[10] = servoID;		// Servo ID
		
		// Compute and set play time in the range 0 - 255
		playTimeByte = playTime / 0.0112;
		if(playTimeByte < 1)
			playTimeByte = 1;
		if(playTimeByte > 255)
			playTimeByte = 255;
		packet[11] = (uint8_t)playTimeByte;
		// --------------------------------------------------------------- //
		
		// Compute the checksum
		computeChecksum((char*)packet, packet[2]);
		
		// Send the packet
		if(error == 0)
			sendData((char*)packet, packet[2], this->usbDevice);
		else
			cout << "ERROR [in ArmController::moveServo]: could not send data to servo." << endl;
	}
	
	return error;
}


/*
 * Limit joint position taking into account mechanical constraints and recommended operation range.
 */
float ArmController::limitServoRotation(uint8_t servoID, float position)
{
	float positionLimited = position;
	int servoFound = 0;
	int k = 0;
	
	
	for(k = 0; k < numArmJoints && servoFound == 0; k++)
	{
		if(servoID == servosID[k])
		{
			servoFound = 1;
			if(position > jointsLimitMax[k])
				positionLimited = jointsLimitMax[k];
			else if(position < jointsLimitMin[k])
				positionLimited = jointsLimitMin[k];
		}
	}
	
	if(servoFound == 0)
		printf("ERROR [in ArmController::limitServoRotation]: servo %d not recognized on arm %d\n", servoID, armID);
	
	
	return positionLimited;
}


/*
 * Open and configure the UART interface, giving as output the file descriptor (-1 in case of error)
 */	
int ArmController::createSerialInterface(const string &uartDeviceName, speed_t speed)
{
	struct termios tty;
	int usbDevice = -1;
	int error = 0;
	
	
	usbDevice = open(uartDeviceName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if(usbDevice <= 0)
	{
		error = 1;
		cout << "ERROR: [in TaskManager::createSerialInterface] could no open USB-to-UART device associated to left arm. Try running with sudo." << endl;
	}
	else
	{
		memset(&tty, 0, sizeof(struct termios));
		if(tcgetattr(usbDevice, &tty) != 0)
		{
			error = 1;
			close(usbDevice);
			cout << "ERROR: [in TaskManager::createSerialInterface] could not get attributes from USB device." << endl;
		}
		else
		{
			// Set baudrate
			cfsetospeed (&tty, speed);
			cfsetispeed (&tty, speed);
			
			// Set other attirbutes
			tty.c_cc[VMIN] = 0;		// read doesn't block
			tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout
			
			tty.c_cflag &=  ~PARENB;	// Make 8n1
			tty.c_cflag &=  ~CSTOPB;
			tty.c_cflag &=  ~CSIZE;
			tty.c_cflag |=  CS8;
			tty.c_cflag &=  ~CRTSCTS;	// no flow control
			tty.c_lflag = 0;			// no signaling chars, no echo, no canonical processing
			tty.c_oflag = 0; 			// no remapping, no delays
			tty.c_cflag |=  CREAD | CLOCAL;			// turn on READ & ignore ctrl lines
			tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// turn off s/w flow ctrl
			tty.c_cflag |=  CREAD | CLOCAL;	// turn on READ & ignore ctrl lines
			tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// make raw
			tty.c_oflag &= ~OPOST;		// make raw
	
			// Ensures that special characters are not interpreted (CR character)
			cfmakeraw(&tty);
	
			// Flush port and apply attributes
			tcflush(usbDevice, TCIFLUSH);
			if(tcsetattr(usbDevice, TCSANOW, &tty) != 0)
			{
				error = 1;
				close(usbDevice);
				cout << "ERROR: [in TaskManager::createSerialInterface] could not apply attributes on USB device." << endl;
			}
		}
	}
	
	if(error != 0)
		usbDevice = -1;
	
	
	return usbDevice;
}


/*
 * Compute the checksum 1 and checksum 2 of the packet passed as first argument with the specified length.
 */
void ArmController::computeChecksum(char * packet, uint8_t packetSize)
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


/*
 * Send data through the serial port.
 */
int ArmController::sendData(char * data, uint8_t dataLength, int usbDevice)
{
	ssize_t bytesSent = 0;
	int error = 0;
	
	
	// Send data
	bytesSent = write(usbDevice, data, dataLength);
	if(bytesSent <= 0)
	{
		error = 1;
		cout << "ERROR: [in ArmController::sendData] could not send any data" << endl;
	}
	else if(bytesSent < dataLength)
	{
		error = 1;
		cout << "ERROR: [in ArmController::sendData] could not send all data" << endl;
	}

	
	return error;
}

