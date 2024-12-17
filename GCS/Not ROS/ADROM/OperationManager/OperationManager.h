/*
 * Copyright (c) 2024 Alejandro Suarez, asuarezfm@us.es
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * The distribution of this source code is not allowed without the consent of the author
 */

#ifndef OPERATIONMANAGER_H_
#define OPERATIONMANAGER_H_


// Standard library
#include <iostream>
#include <stdio.h>
#include <sys/time.h>


// Specific library
#include "../ModuleInterface/ModuleInterface.h"


// Constant definition
#define MODULE_ID_VOCOR			1
#define MODULE_ID_LiCAS			2
#define MODULE_ID_VISION		3
#define MODULE_ID_ARDUPILOT		4
#define MODULE_ID_PLANNER		5
#define MODULE_ID_TARGET		6

using namespace std;


class OperationManager
{
public:

	typedef struct
	{
		float pos[3];
		float quat[4];
	} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;

	/***************** PUBLIC VARIABLES *****************/
	
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	OperationManager();

	/*
	 * Destructor
	 * */
	virtual ~OperationManager();
	
	/*
	 * Open interfaces.
	 */
	int openInterfaces();
	
	/*
	 * Get operation parameters (Supply Point, Delivery Point, packet ID)
	 */
	int getOperationParams();
	
	/*
	 * Start operation
	 */
	int startOperation();
	
	/*
	 * Terminate operation.
	 */
	int terminateOperation();
	
	/*
	 * Close interfaces.
	 */
	int closeInterfaces();
	

private:
	/***************** PRIVATE VARIABLES *****************/

	ModuleInterface * mifVoCoR;
	ModuleInterface * mifLiCAS;
	ModuleInterface * mifVision;
	ModuleInterface * mifArduPilot;
	ModuleInterface * mifPlanner;
	ModuleInterface * mifTarget;
	
	string supplyPointName;
	string deliveryPointName;
	int parcelID;
	
	
	/***************** PRIVATE METHODS *****************/
	
	int flagOperationComplete;
	int flagAbortOperation;
	int flagTerminateOperation;
	

};

#endif


