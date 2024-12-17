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


const uint16_t TaskManager::GCS_CODE_NOP 						= 0;
const uint16_t TaskManager::GCS_CODE_GO_TO_REST_POSITION 		= 1;
const uint16_t TaskManager::GCS_CODE_GO_TO_OPERATION_POSITION 	= 2;
const uint16_t TaskManager::GCS_CODE_VISUAL_SERVOING 			= 3;
const uint16_t TaskManager::GCS_CODE_TELEOPERATION				= 4;
const uint16_t TaskManager::GCS_CODE_SERVOS_CALIBRATION			= 10;
const uint16_t TaskManager::GCS_CODE_JOINTS_IDENTIFICATION		= 11;
const uint16_t TaskManager::GCS_CODE_DISABLE_TORQUE_CONTROL		= 49;
const uint16_t TaskManager::GCS_CODE_ENABLE_TORQUE_CONTROL		= 50;
const uint16_t TaskManager::EXTERNAL_JOINT_CONTROL_REF			= 101;


/*
 * Constructor
 * */
TaskManager::TaskManager()
{
	int k = 0;
	
	
	// Init operation code
	gcsCode = 0;
	gcsCodeReceivedFlag = 0;
	
	// Set to zero the end thread signal
	endThreadsSignal = 0;
	
	// Socket for publishing the signal
	socketSignalPublisher = -1;

	// Set to zero the vision module data
	visionModuleData.validL = 0;
	visionModuleData.validR = 0;
	for(k = 0; k < 3; k++)
	{
		visionModuleData.pL[k] = 0;
		visionModuleData.pR[k] = 0;
	}
	
	// Create the module interface
	miLiCAS = new ModuleInterface(1, "LiCAS_Module");
	
	// Create the data log files
	// this->createLogFiles();
}


/*
 * Destructor
 * */
TaskManager::~TaskManager()
{
}

		
/*
 * Start task execution, taking GCS code from thread function
 */
void TaskManager::startTaskExecutionFromGCS()
{
	int requestCode = 0;
	int requestOption = 0;
	
	while(this->endThreadsSignal == 0 && this->gcsCode >= 0)
	{
		if(this->gcsCode > 0 && this->gcsCodeReceivedFlag != 0)
		{
			gcsCodeReceivedFlag = 0;
			this->updateTaskExecution(this->gcsCode);
		}
		if(this->miLiCAS->isRequestReceived() != 0)
		{
			cout << "ADROM request received." << endl;
			this->miLiCAS->getRequest(requestCode, requestOption);
			this->gcsCode = requestCode;
			printf("Requested operation code: %d\n", requestCode);
			this->updateTaskExecution(requestCode);
			requestCode = 0;
			requestOption = 0;
		}
		usleep(20000);	// Wait 50 ms
	}
}
	
	
/*
 * Termiante task execution, closing interfaces and log files
 */
void TaskManager::terminateTaskExecution()
{
	// Close interface
	miLiCAS->closeInterface();
	
	// Close log files
	this->closeLogFiles();
}

	
/*
 * Init the dual arm manipulator
 */
int TaskManager::initDualArmManipulator(const vector<string> &uartDeviceName)
{
	int error = 0;
	
	
	// Check if the number of USB-UART devices is correct
	if(uartDeviceName.size() < 2)
	{
		error = 1;
		cout << "ERROR [in TaskManager::initDualArmManipulator]: insufficient number of USB-UART devices. Minimum 2 for the dual arm." << endl;
	}
	else
	{
		// Create the instances to left and right arms
		leftArmController = new ArmController(1);
		rightArmController = new ArmController(2);
		
		// Init the left arm instance
		servoIDs_L[0] = 1; 				servoIDs_L[1] = 4;
		servoModels_L[0] = 602;			servoModels_L[1] = 402;
		signCorrections_L[0] = 1;		signCorrections_L[1] = -1;
		offsetCorrections_L[0] = -5.0;	offsetCorrections_L[1] = -12.0;
		maxJointLimits_L[0] = 90;		maxJointLimits_L[1] = 135;
		minJointLimits_L[0] = -60;		minJointLimits_L[1] = -135;
		error += leftArmController->init(NUM_ARM_JOINTS, servoIDs_L, servoModels_L, signCorrections_L, offsetCorrections_L, maxJointLimits_L, minJointLimits_L, uartDeviceName[0]);
		usleep(10000);
		if(error != 0)
			cout << "ERROR [in TaskManager::initDualArmManipulator]: could not init left arm." << endl;
		
		// Init the right arm instance
		servoIDs_R[0] = 1; 				servoIDs_R[1] = 4;
		servoModels_R[0] = 602;			servoModels_R[1] = 402;
		signCorrections_R[0] = -1;		signCorrections_R[1] = 1;
		offsetCorrections_R[0] = 5.0;	offsetCorrections_R[1] = -7.0;
		maxJointLimits_R[0] = 90;		maxJointLimits_R[1] = 135;
		minJointLimits_R[0] = -60;		minJointLimits_R[1] = -135;
		error += rightArmController->init(NUM_ARM_JOINTS, servoIDs_R, servoModels_R, signCorrections_R, offsetCorrections_R, maxJointLimits_R, minJointLimits_R, uartDeviceName[1]);
		usleep(10000);
		if(error != 0)
			cout << "ERROR [in TaskManager::initDualArmManipulator]: could not init left arm." << endl;
	}
	
	
	return error;
}
	

/*
 * Init external interfaces: 3DConnexion mouse for teleoperation, visual marker position, ...
 */
int TaskManager::initExternalInterfaces()
{
	int error = 0;
	
	
	// Init GCS interface
	gcsThread = thread(&TaskManager::receiveGCSCommand, this);
	
	// Init the module interface
	miLiCAS->openUDPSocket("192.168.0.171", 25001, 24001);
	
	// Init the signal publisher socket
	socketSignalPublisher = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socketSignalPublisher < 0)
    {
    	cout << endl << "ERROR [TaskManager::initExternalInterfaces]: could not open socket." << endl;
    	error = 1;
   	}
		
   	host = gethostbyname(IP_ADDRESS_SIGNAL_HANDLER);
    if(host == NULL)
	{
	    cout << "ERROR [TaskManager::initExternalInterfaces]: could not get host by name" << endl;
	    error = 1;
	}
	
	// Set the address of the host
	if(error == 0)
	{
		bzero((char*)&addrHost, sizeof(struct sockaddr_in));
		addrHost.sin_family = AF_INET;
		bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
		addrHost.sin_port = htons(UDP_PORT_SIGNAL_HANDLER);
	}
	else
		close(socketSignalPublisher);	
	
	// Init the 3DConnexion interface for teleoperation
	joystickTeleopThread = thread(&TaskManager::receive3DConnexionMouseData, this);
	joystickTeleopThread.detach();
	
	// Init the visual data sensor for object manipulation
	visualDataThread = thread(&TaskManager::receiveVisionModuleData, this);
	visualDataThread.detach();

	
	return error;
}


/*
 * Update task status
 */
void TaskManager::updateTaskExecution(int gcsCode)
{
	switch(gcsCode)
	{
		case GCS_CODE_NOP:
			// No operation code. Wait 100 ms
			usleep(100000);
			break;
			
		//////////////////////////// ARMS OPERATION CODES ////////////////////////////
		case GCS_CODE_GO_TO_REST_POSITION:
			this->goToRestPosition();
			break;

		case GCS_CODE_GO_TO_OPERATION_POSITION:
			this->goToOperationPosition();
			break;
		
		case GCS_CODE_TELEOPERATION:
			this->teleoperation3DConnexion();
			break;
		
		case GCS_CODE_VISUAL_SERVOING:
			cout << "EXECUTING VISUAL SERVOING" << endl;
			this->visualServoingGrasping();
			break;
			
		case GCS_CODE_DISABLE_TORQUE_CONTROL:
			this->leftArmController->armTorqueControl(0);
			this->rightArmController->armTorqueControl(0);
			break;
				
		case GCS_CODE_ENABLE_TORQUE_CONTROL:
			this->leftArmController->armTorqueControl(1);
			this->rightArmController->armTorqueControl(1);
			break;
				
		//////////////////////////// DEBUG OPERATION CODES ////////////////////////////
			
		case GCS_CODE_SERVOS_CALIBRATION:
			this->dualArmServosCalibration();
			break;
			
		case GCS_CODE_JOINTS_IDENTIFICATION:
			this->dualArmJointsIdentification();
			break;
				
		//////////////////////////// EXTERNAL CONTROL CODES ////////////////////////////
		
		case EXTERNAL_JOINT_CONTROL_REF:
			this->externalJointReferenceControl();
			break;
		
		default:
			if(gcsCode < 0)
			{
				// Terminate arm controllers
				endThreadsSignal = 1;
				this->leftArmController->endServoThread();
				this->rightArmController->endServoThread();
				usleep(10000);
			}
			else
			{
				cout << "WARNING: unrecognized GCS code." << endl;
				usleep(100000);
			}
			break;
	}
}


/*
 *  Ask user to specify the desired rotation angle for the servo of both left and right arms
 */
void TaskManager::dualArmServosCalibration()
{
	float playTime = 1;
	float angle = 0;
	int armID = 0;
	int servoID = 0;
	int k = 0;
	
	
	printf("\t\t\t--- ARM SERVOS CALIBRATION ---\n");
	printf("Note: specified angles are directly applied to servos, with no sign/offset correction\n");

	do
	{
		printf("\n");
		printf("\t1. Calibrate left arm servos\n");
		printf("\t2. Calibrate right arm servos\n");
		printf("\t0. Exit\n");
		printf("\t\t- Option: ");
		scanf("%d", &armID);
		
		if(armID == 1 || armID == 2)
		{
			do
			{
				printf("\n");
				for(k = 0; k < NUM_ARM_JOINTS; k++)
					if(armID == 1)
						printf("\t%d. Calibrate servo %d\n", k+1, servoIDs_L[k]);
					else
						printf("\t%d. Calibrate servo %d\n", k+1, servoIDs_R[k]);
				printf("\t\t- Option: ");
				scanf("%d", &servoID);
				
				printf("\n");
				printf("\t\t- Angle: ");
				scanf("%f", &angle);
						
				if(armID == 1)
					leftArmController->moveServo(servoIDs_L[servoID], angle, playTime);
				else
					rightArmController->moveServo(servoIDs_R[servoID], angle, playTime);
					
			} while(servoID != 0);
		}
	} while(armID != 0);
}


/*
 *  Moves the different joints of the arms: shoulder pitch, shoulder roll, shoulder yaw and elbow pitch
 */
void TaskManager::dualArmJointsIdentification()
{
	float playTime = 1;
	float angle = 0;
	int armID = 0;
	int servoID = 0;
	int k = 0;
	
	
	printf("\t\t\t--- ARM JOINTS IDENTIFICATION ---\n");
	printf("Note: specified angles are directly applied to servos, with no sign/offset correction\n");

	do
	{
		printf("\n");
		printf("\t1. Identify left arm joints\n");
		printf("\t2. Identify right arm joints\n");
		printf("\t0. Exit\n");
		printf("\t\t- Option: ");
		scanf("%d", &armID);
		
		if(armID == 1 || armID == 2)
		{
			do
			{
				printf("\n");
				for(k = 0; k < NUM_ARM_JOINTS; k++)
					if(armID == 1)
						printf("\t%d. Identify joint %d\n", k+1, servoIDs_L[k]);
					else
						printf("\t%d. Identify joint %d\n", k+1, servoIDs_R[k]);
				printf("\t\t- Option: ");
				scanf("%d", &servoID);
				
				printf("\n");
				printf("\t\t- Angle: ");
				scanf("%f", &angle);
						
				if(armID == 1)
					leftArmController->moveJoint(servoIDs_L[servoID], angle, playTime);
				else
					rightArmController->moveJoint(servoIDs_R[servoID], angle, playTime);
					
			} while(servoID != 0);
		}
	} while(armID != 0);
}


/*
 * Move the dual arm system to the rest position
 */
void TaskManager::goToRestPosition()
{
	float qL_ref[NUM_ARM_JOINTS] = {0.0, 0.0};
	float qR_ref[NUM_ARM_JOINTS] = {0.0, 0.0};
	float playTime = 2;
	
	
	cout << "Moving arms to rest position..." << endl;
	this->leftArmController->moveJoints(qL_ref, playTime);
	this->rightArmController->moveJoints(qR_ref, playTime);
	
	miLiCAS->sendMessage(TaskManager::GCS_CODE_GO_TO_REST_POSITION, 0, NULL, 0);
}


/*
 * Move the dual arm system to the operation position
 */
void TaskManager::goToOperationPosition()
{
	float qL_ref[NUM_ARM_JOINTS] = {60.0, -135.0};
	float qR_ref[NUM_ARM_JOINTS] = {60.0, -135.0};
	float playTime = 2;
	
	
	cout << "Moving arms to operation position..." << endl;
	this->leftArmController->moveJoints(qL_ref, playTime);
	this->rightArmController->moveJoints(qR_ref, playTime);
	
	miLiCAS->sendMessage(TaskManager::GCS_CODE_GO_TO_OPERATION_POSITION, 0, NULL, 0);
}

	
/*
 * Teleoperation with 3DConnexon mouse
 */
void TaskManager::teleoperation3DConnexion()
{
	struct timeval tini;
	struct timeval tend;
	double delta_t = 0;
	const float speedRef = 0.25;
	float pJoystick[3] = {0.0, 0.0, 0.0};
	float pL[3] = {0.0, 0.0, 0.0};
	float pR[3] = {0.0, 0.0, 0.0};
	float pL_ref[3] = {0.0, 0.0, 0.0};
	float pR_ref[3] = {0.0, 0.0, 0.0};
	float eL[3] = {0.0, 0.0, 0.0};
	float eR[3] = {0.0, 0.0, 0.0};
	float eL_norm = 0;
	float eR_norm = 0;
	int k = 0;
	
	// Init TCP reference position
	leftArmController->getCartesianPosition(pL);
	rightArmController->getCartesianPosition(pR);
	for(k = 0; k < 3; k++)
	{
		pL_ref[k] = pL[k];
		pR_ref[k] = pR[k];
	}
	
	while(gcsCode == TaskManager::GCS_CODE_TELEOPERATION && this->endThreadsSignal == 0)
	{
		// Get time stamp at the begining of the loop
		gettimeofday(&tini, NULL);
		
		// Get current TCP position
		leftArmController->getCartesianPosition(pL);
		rightArmController->getCartesianPosition(pR);
		
		// Update the position reference
		pJoystick[0] = speedRef/DUAL_ARM_CONTROL_RATE*this->joystickTeleopData.x;
		pJoystick[1] = speedRef/DUAL_ARM_CONTROL_RATE*this->joystickTeleopData.y;
		pJoystick[2] = speedRef/DUAL_ARM_CONTROL_RATE*this->joystickTeleopData.z;
		for(k = 0; k < 3; k++)
		{
			// pL_ref[k] = pL[k] + pJoystick[k];
			// pR_ref[k] = pR[k] + pJoystick[k];
			pL_ref[k] += pJoystick[k];
			pR_ref[k] += pJoystick[k];
		}
		
		// Compute TCP Cartesian position error (unit vector, normalized)
		eL_norm = 0;
		eR_norm = 0;
		for(k = 0; k < 3; k++)
		{
			eL[k] = pL_ref[k] - pL[k];
			eR[k] = pR_ref[k] - pR[k];
			eL_norm += eL[k]*eL[k];
			eR_norm += eR[k]*eR[k];
		}
		eL_norm = sqrt(eL_norm);
		eR_norm = sqrt(eR_norm);
		
		// Move arms to reference position
		leftArmController->goToCartesianPosition_q2(2.0/DUAL_ARM_CONTROL_RATE, pL_ref, 0);
		rightArmController->goToCartesianPosition_q2(2.0/DUAL_ARM_CONTROL_RATE, pR_ref, 0);
		
		// Get time stamp at the end of the loop
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_sec - tini.tv_sec);
		if(delta_t > 1/DUAL_ARM_CONTROL_RATE)
			printf("WARNING [in TaskManager::teleoperation3DConnexion]: control period exceeded: %lf > %lf\n", delta_t, 1/DUAL_ARM_CONTROL_RATE);
		else
			usleep((useconds_t)(1e6*(1/DUAL_ARM_CONTROL_RATE - delta_t)));
	}
}

	
/*
 * Visual servoing and object grasping
 */
void TaskManager::visualServoingGrasping()
{
	ofstream outDataFile;
	struct timeval tini;
	struct timeval tend;
	double delta_t = 0;
	struct timeval t_task_ini;
	struct timeval t_task_act;
	double t_actual = 0;
	double t_update = 0;
	extern struct timeval t0;
	struct timeval t1;
	double t = 0;
	double t_operation = 0;
	const float speedRef = 0.4;
	const float reachLimit = 0.5;
	const float graspingThreshold = 0.085;
	float pJoystick[3] = {0.0, 0.0, 0.0};
	float qL_ref[NUM_ARM_JOINTS] = {45.0, -120.0};
	float qR_ref[NUM_ARM_JOINTS] = {45.0, -120.0};
	float pL[3] = {0.0, 0.0, 0.0};
	float pR[3] = {0.0, 0.0, 0.0};
	float pL_ref[3] = {0.0, 0.0, 0.0};
	float pR_ref[3] = {0.0, 0.0, 0.0};
	float pL_vis[3] = {0.0, 0.0, 0.0};
	float pR_vis[3] = {0.0, 0.0, 0.0};
	float eL[3] = {0.0, 0.0, 0.0};
	float eR[3] = {0.0, 0.0, 0.0};
	float normL = 0;
	float normR = 0;
	float eL_norm = 0;
	float eR_norm = 0;
	int graspedL = 0;
	int graspedR = 0;
	int k = 0;
	
	
	// Open output data file
	outDataFile.open("DataFile_VisualServoing.txt");
	
	// Move arms to initial pose
	/*
	cout << "Moving arms to operation position...";
	this->leftArmController->moveJoints(qL_ref, 2.0);
	this->rightArmController->moveJoints(qR_ref, 2.0);
	usleep(2250000);
	cout << "Done" << endl;
	*/

	// Init TCP reference position
	leftArmController->getCartesianPosition(pL);
	rightArmController->getCartesianPosition(pR);
	for(k = 0; k < 3; k++)
	{
		pL_ref[k] = pL[k];
		pR_ref[k] = pR[k];
	}

	// Get time stamp at the begining of the task
	gettimeofday(&t_task_ini, NULL);
	
	// while(gcsCode == TaskManager::GCS_CODE_VISUAL_SERVOING && this->endThreadsSignal == 0 && (graspedL == 0 || graspedR == 0))
	while(gcsCode == TaskManager::GCS_CODE_VISUAL_SERVOING && this->endThreadsSignal == 0 && t_operation < 2.0)
	{
		// Get time stamp at the begining of the loop
		gettimeofday(&tini, NULL);

		// Get current task time stamp
		gettimeofday(&t_task_act, NULL);
		t_actual = (t_task_act.tv_sec - t_task_ini.tv_sec) + 1e-6*(t_task_act.tv_usec - t_task_ini.tv_usec);
		t_operation = t_actual;
		
		// Get current TCP position
		leftArmController->getCartesianPosition(pL);
		rightArmController->getCartesianPosition(pR);
		
		// Compute TCP Cartesian position error (unit vector, normalized)
		eL_norm = 0;
		eR_norm = 0;
		for(k = 0; k < 3; k++)
		{
			pL_vis[k] = this->visionModuleData.pL[k];
			pR_vis[k] = this->visionModuleData.pR[k];
			if(k == 0)
			{
				pL_vis[k] += 0.05;
				pR_vis[k] += 0.05;
			}
			if(k == 2)
			{
				pL_vis[k] += 0.00;
				pR_vis[k] += 0.00;
			}
			eL[k] = pL_vis[k] - pL[k];
			eR[k] = pR_vis[k] - pR[k];
			eL_norm += eL[k]*eL[k];
			eR_norm += eR[k]*eR[k];
		}
		eL_norm = sqrt(eL_norm);
		eR_norm = sqrt(eR_norm);
		
		// Print visual position reference
		// printf("Left marker: {%.1f, %.1f, %.1f} [cm]\n", 100*pL_vis[0], 100*pL_vis[1], 100*pL_vis[2]);
		// printf("Right marker: {%.1f, %.1f, %.1f} [cm]\n", 100*pR_vis[0], 100*pR_vis[1], 100*pR_vis[2]);

		// Update the position reference
		for(k = 0; k < 3; k++)
		{
			pL_ref[k] = pL[k] + speedRef / DUAL_ARM_CONTROL_RATE * eL[k]/(eL_norm + 0.001);
			pR_ref[k] = pR[k] + speedRef / DUAL_ARM_CONTROL_RATE * eR[k]/(eR_norm + 0.001);
			// pL_ref[k] = this->visionModuleData.pL[k];
			// pR_ref[k] = this->visionModuleData.pR[k];
		}
		
		// Move arms to reference position if this is within its reach
		normL = sqrt(pL_ref[0]*pL_ref[0] + pL_ref[1]*pL_ref[1] + pL_ref[2]*pL_ref[2]);
		normR = sqrt(pR_ref[0]*pR_ref[0] + pR_ref[1]*pR_ref[1] + pR_ref[2]*pR_ref[2]);
		if(this->visionModuleData.pL[0] >= 0.1 && this->visionModuleData.pL[0] <= 0.45 && normL < reachLimit)
		{
			// leftArmController->goToCartesianPosition_q2(2.0/DUAL_ARM_CONTROL_RATE, pL_ref, 0);
		// if(this->visionModuleData.pR[0] >= 0.1 && this->visionModuleData.pR[0] <= 0.45 && normR < reachLimit)
			// rightArmController->goToCartesianPosition_q2(2.0/DUAL_ARM_CONTROL_RATE, pR_ref, 0);
			if(t_actual - t_update >= 0.7)
			{
				t_update = t_actual;
				leftArmController->goToCartesianPosition_q2(1.4, pL_vis, 0);
				rightArmController->goToCartesianPosition_q2(1.4, pR_vis, 0);
			}
		}
		else
			cout << "WARNING: Non reachable point for grasping." << endl;
		
		// Reset the vision module valid flag once the sample is used	
		this->visionModuleData.validL = 0;
		this->visionModuleData.validR = 0;
		
		// printf("Grasping error LR = {%.1f, %.1f} [cm]\n", 100*eL_norm, 100*eR_norm);
		
		// Check if object is grasped
		if(eL_norm < graspingThreshold)
			graspedL = 1;
		else
			graspedL = 0;
		if(eR_norm < graspingThreshold)
			graspedR = 1;
		else
			graspedR = 0;
		
		// Get current time stamp
		gettimeofday(&t1, NULL);
		t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_sec - t0.tv_sec);
		
		// Save data on file
		if(outDataFile.is_open())
		{
			outDataFile << t << "\t";
			outDataFile << pL_ref[0] << "\t" << pL_ref[1] << "\t" << pL_ref[2] << "\t";
			outDataFile << pR_ref[0] << "\t" << pR_ref[1] << "\t" << pR_ref[2] << "\t";
			outDataFile << pL[0] << "\t" << pL[1] << "\t" << pL[2] << "\t";
			outDataFile << pR[0] << "\t" << pR[1] << "\t" << pR[2] << "\t";
			outDataFile << endl;
		}
		
		// Get time stamp at the end of the loop
		gettimeofday(&tend, NULL);
		delta_t = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_sec - tini.tv_sec);
		if(delta_t > 1/DUAL_ARM_CONTROL_RATE)
			printf("WARNING [in TaskManager::teleoperation3DConnexion]: control period exceeded: %lf > %lf\n", delta_t, 1/DUAL_ARM_CONTROL_RATE);
		else
			usleep((useconds_t)(1e6*(1/DUAL_ARM_CONTROL_RATE - delta_t)));
	}
	
	// Retrieve object by moving grippers upwards 25 cm
	leftArmController->getCartesianPosition(pL_ref);
	rightArmController->getCartesianPosition(pR_ref);
	pL_ref[2] += 0.25;
	pR_ref[2] += 0.25;
	leftArmController->goToCartesianPosition_q2(1.0, pL_ref, 0);
	rightArmController->goToCartesianPosition_q2(1.0, pR_ref, 0);
	usleep(1250000);
	pL_ref[0] -= 0.05;
	pR_ref[0] -= 0.05;
	// pL_ref[2] = -0.15;
	// pR_ref[2] = -0.15;
	leftArmController->goToCartesianPosition_q2(1.0, pL_ref, 0);
	rightArmController->goToCartesianPosition_q2(1.0, pR_ref, 0);
	usleep(1250000);
	
	// Send message indicating grasping complete
	miLiCAS->sendMessage(TaskManager::GCS_CODE_VISUAL_SERVOING, 0, NULL, 0);
	
	// Move arms to retracted pose
	cout << "Object grasped" << endl;
	// cout << "Moving arms to retracted position...";
	// this->leftArmController->moveJoints(qL_ref, 2.0);
	// this->rightArmController->moveJoints(qR_ref, 2.0);
	// usleep(2250000);
	// cout << "Done" << endl;
	
	// Close output data file
	if(outDataFile.is_open())
		outDataFile.close();
}

	
/*
 * Create the log files. Return 0 if no error occurred.
 */
int TaskManager::createLogFiles()
{
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char fileName[256];


	sprintf(fileName, "Log_%d-%d-%d_%dh%dm%ds_LeftArm.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	leftArmDataLogFile.open(fileName);
	sprintf(fileName, "Log_%d-%d-%d_%dh%dm%ds_RightArm.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	rightArmDataLogFile.open(fileName);
	sprintf(fileName, "Log_%d-%d-%d_%dh%dm%ds_Position.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	rightArmDataLogFile.open(fileName);
	sprintf(fileName, "Log_%d-%d-%d_%dh%dm%ds_VisionData.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	leftArmDataLogFile.open(fileName);
	sprintf(fileName, "Log_%d-%d-%d_%dh%dm%ds_GCS.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	gcsDataLogFile.open(fileName);


	return 0;
}

	
/*
 * Close the log files. Return 0 if no error occurred.
 */
void TaskManager::closeLogFiles()
{
	leftArmDataLogFile.close();
	rightArmDataLogFile.close();
	positionDataLogFile.close();
	visionDataLogFile.close();
	gcsDataLogFile.close();
}

	
/*
 * Return -1, 0, 1 depending on the value
 */
int TaskManager::signDouble(double value)
{
	int signValue = 1;
	
	if(value == 0)
		signValue = 0;
	if(value < 0)
		signValue = -1;
	
	return signValue;
}


