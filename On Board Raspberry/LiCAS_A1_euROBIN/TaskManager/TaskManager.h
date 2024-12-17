/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: TaskManager.h
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_


// Standard library
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// Specific library
#include "../ArmController/ArmController.h"
#include "../ModuleInterface/ModuleInterface.h"
#include "../Structures/Structures.h"

// Constant definition
#define NUM_ARM_JOINTS 				2		// Num of joints of the each arm
#define DUAL_ARM_CONTROL_RATE		50.0F	// Control rate in Hz
#define IP_ADDRESS_SIGNAL_HANDLER	"127.0.0.1"
#define UDP_PORT_SIGNAL_HANDLER		34000
#define UDP_PORT_GCS				22000	// UDP port for receiving GCS commands
#define UDP_PORT_3DCONNEXION		22001	// UDP port for receiving 3DConnexion mouse data
#define UDP_PORT_VISION_MODULE		32002	// UDP port for receiving visual grasping data

#define STATE_SIGNAL_MODULE_ID				1	// Unique ID of the LiCAS control software
#define STATE_SIGNAL_ARMS_ZERO_POSE_REACHED	1	// Signal generated when arms reach the zero pose
#define STATE_SIGNAL_ARMS_OP_POSE_REACHED	2	// Signal generated when arms reach the operation pose
#define STATE_SIGNAL_OBJECT_GRASPED			3	// Signal corresponding to the object grasped


// Namespaces
using namespace std;


class TaskManager
{
	// Data packet received from the Ground Control Station through an UDP socket
	typedef struct
	{
		char header[3];		// "GCS" character sequence
		int code;
	} GCS_PACKET;
	
	// Structure definition
	typedef struct
	{
		char header[3];		// "TOP" (tele-operation) character sequence
		float x;
		float y;
		float z;
		float roll;
		float pitch;
		float yaw;
		int buttons[2];;
	} DATA_PACKET_3DCONNEXION;
	
	typedef struct
	{
		char header[3];		// "VDP" (visual data packet) character sequence
		uint8_t validL;
		uint8_t validR;
		float pL[3];
		float pR[3];
	} __attribute__((packed)) DATA_PACKET_VISION_MODULE;
	
	typedef struct
	{
		char header[3];		// "SIG" (state signal data packet) character sequence
		uint8_t moduleID;	// Identifier of the module generating the signal (unique for each module)
		uint8_t signal;		// Signal code
		uint8_t signal2;	// Signal code 2
	} __attribute__((packed)) DATA_PACKET_STATE_SIGNAL;


public:

	/***************** PUBLIC VARIABLES *****************/
	
	static const uint16_t GCS_CODE_NOP;
	static const uint16_t GCS_CODE_GO_TO_REST_POSITION;
	static const uint16_t GCS_CODE_GO_TO_OPERATION_POSITION;
	static const uint16_t GCS_CODE_VISUAL_SERVOING;
	static const uint16_t GCS_CODE_TELEOPERATION;
	static const uint16_t GCS_CODE_SERVOS_CALIBRATION;
	static const uint16_t GCS_CODE_JOINTS_IDENTIFICATION;
	static const uint16_t GCS_CODE_DISABLE_TORQUE_CONTROL;
	static const uint16_t GCS_CODE_ENABLE_TORQUE_CONTROL;
	static const uint16_t EXTERNAL_JOINT_CONTROL_REF;
	
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	TaskManager();

	/*
	 * Destructor
	 * */
	virtual ~TaskManager();
	
	
	/*
	 * Start task execution, taking GCS code from thread function
	 */
	void startTaskExecutionFromGCS();
	
	/*
	 * Termiante task execution, closing interfaces and log files
	 */
	void terminateTaskExecution();
	
	/*
	 * Init the dual arm manipulator
	 */
	int initDualArmManipulator(const vector<string> &uartDeviceName);
	
	/*
	 * Init external interfaces: 3DConnexion mouse for teleoperation, visual marker position, ...
	 */
	int initExternalInterfaces();

	/*
	 * Enable/Disables servos torque (0: disable, 1: enable).
	 */
	void enableServosTorqueControl(uint8_t servosTorqueMode);

	/*
	 * Update task status
	 */
	void updateTaskExecution(int gcsCode);

	/*
	 *  Ask user to specify the desired rotation angle for the servo of both left and right arms
	 */
	void dualArmServosCalibration();
	
	/*
	 *  Moves the different joints of the arms: shoulder pitch, shoulder roll, shoulder yaw and elbow pitch
	 */
	void dualArmJointsIdentification();
	
	/*
	 * Move the dual arm system to the rest position
	 */
	void goToRestPosition();

	/*
	 * Move the dual arm system to the operation position
	 */
	void goToOperationPosition();

	/*
	 * Teleoperation with 3DConnexon mouse
	 */
	void teleoperation3DConnexion();
	
	/*
	 * Visual servoing and object grasping
	 */
	void visualServoingGrasping();
	
	/*
	 * Take control references from external source
	 */
	void externalJointReferenceControl();
	
	/*
	 * Executes a sequence of rotations
	 */
	void executeRotationSequence();

	/*
	 * Create/close the log files. Return 0 if no error occurred.
	 */
	int createLogFiles(); 
	void closeLogFiles();
	
private:
	/***************** PRIVATE VARIABLES *****************/
	
	ArmController * leftArmController = NULL;
	ArmController * rightArmController = NULL;
	
	ofstream leftArmDataLogFile;
	ofstream rightArmDataLogFile;
	ofstream positionDataLogFile;
	ofstream visionDataLogFile;
	ofstream gcsDataLogFile;
	
	int gcsCode;
	int gcsCodeReceivedFlag;
	
	uint8_t servoIDs_L[NUM_ARM_JOINTS];
	int servoModels_L[NUM_ARM_JOINTS];
	int signCorrections_L[NUM_ARM_JOINTS];
	float offsetCorrections_L[NUM_ARM_JOINTS];
	float maxJointLimits_L[NUM_ARM_JOINTS];
	float minJointLimits_L[NUM_ARM_JOINTS];
	
	uint8_t servoIDs_R[NUM_ARM_JOINTS];
	int servoModels_R[NUM_ARM_JOINTS];
	int signCorrections_R[NUM_ARM_JOINTS];
	float offsetCorrections_R[NUM_ARM_JOINTS];
	float maxJointLimits_R[NUM_ARM_JOINTS];
	float minJointLimits_R[NUM_ARM_JOINTS];
	
	
	int socketSignalPublisher;
	struct sockaddr_in addrHost;
    struct hostent * host;
	
	thread gcsThread;
	thread joystickTeleopThread;
	thread visualDataThread;
	
	DATA_PACKET_3DCONNEXION joystickTeleopData;
	DATA_PACKET_VISION_MODULE visionModuleData;	// Vision module data in dual arm reference frame {0}
	
	ModuleInterface * miLiCAS;
	
	int endThreadsSignal;
	
	/***************** PRIVATE METHODS *****************/

	/*
	 * Thread for receiving GCS commands
	 */
	void receiveGCSCommand();

	/*
	 * Thread for receiving the 3DConnexion joystick references
	 */
	void receive3DConnexionMouseData();

	/*
	 * Send signal to signal manager through UDP interface
	 */
	int sendSignalUDPSocket(uint8_t moduleID, uint8_t signal, uint8_t signal2);
	
	/*
	 * Receive data from visual sensor
	 */
	void receiveVisionModuleData();

	/*
	 * Return -1, 0, 1 depending on the value
	 */
	int signDouble(double value);	

};

#endif /* TASKMANAGER_H_ */


