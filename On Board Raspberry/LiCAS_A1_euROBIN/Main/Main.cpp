/*
 * Copyright (c) 2024 Alejandro Suarez Fernandez-Miranda
 *
 * This source code is part of LiCAS Robotic Arms
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Main.cpp
 */


// Standard library
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>



// Specific library
#include "../TaskManager/TaskManager.h"


// Global variables
struct timeval t0;


// Namespaces
using namespace std;


int main(int argc, char ** argv)
{
	TaskManager taskManager;
	vector<string> uartDeviceName;
	int gcsCode = 0;
	int error = 0;
	
	
	cout << endl;
	cout << "LiCAS Robotic Arms - LiCAS A1 euROBIN" << endl;
	cout << "Author: Alejandro Suarez Fernandez-Miranda" << endl;
	cout << "Date: 9 September 2024" << endl;
	cout << "--------------------------------------------------" << endl;
	cout << endl;
	
	
	if(argc != 3)
	{
		error = 1;
		cout << "ERROR: invalid number of arguments" << endl;
		cout << "Ussage: " << argv[0] << " ttyUSB_LeftArm ttyUSB_RightArm" << endl;
		cout << "USB device name should be like /dev/ttyUSB0" << endl << endl;;
	}
	else
	{
		// Init the dual arm manipulator
		uartDeviceName.push_back(string(argv[1]));
		uartDeviceName.push_back(string(argv[2]));
		error = taskManager.initDualArmManipulator(uartDeviceName);
		if(error == 0)
		{
			taskManager.initExternalInterfaces();
		
		    // Get initial time stamp (global variable, visible by all the threads)
		    gettimeofday(&t0, NULL);
	    
		    // Start task execution from GCS commands
		    taskManager.startTaskExecutionFromGCS();
		}
	}
	
	// Release resources
	taskManager.terminateTaskExecution();
	taskManager.~TaskManager();
	
	
	return error;
}


