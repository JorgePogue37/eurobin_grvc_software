/*
 * Copyright (c) 2024 Alejandro Suarez, asuarezfm@us.es
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Main.cpp
 */


// Standard library
#include <iostream>
#include <string>
#include <stdio.h>


// Specific library
#include "../OperationManager/OperationManager.h"


// Namespaces
using namespace std;


int main(int argc, char ** argv)
{
	OperationManager opm = OperationManager();
	int errorCode = 0;
	
	
	cout << endl;
	cout << "ADROM: Aerial Delivery Robot Operations Manager" << endl;
	cout << "Author: Alejandro Suarez, asuarezfm@us.es" << endl;
	cout << "Date: 6 November 2024" << endl;
	cout << "Organization: GRVC - University of Seville" << endl;
	cout << "--------------------------------------------------" << endl;
	cout << endl;
	
	if(argc != 1)
	{
		errorCode = 1;
		cout << "ERROR [in Main]: invalid number of arguments.\n" << endl;
	}
	else
	{
		// Open module interfaces
		opm.openInterfaces();
		
		// Get operation parameters (Supply Point, Delivery Point, packet ID)
		opm.getOperationParams();
		
		// Start operation
		opm.startOperation();
		
		// Terminate operation
		opm.terminateOperation();
		
		// Close interfaces
		opm.closeInterfaces();
	}
	
	
	return errorCode;
}



