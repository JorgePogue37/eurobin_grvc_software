/*
 * Copyright (c) 2024 Alejandro Suarez, asuarezfm@us.es
 *
 * University of Seville - Robotics, Vision and Control Group
 *
 * The distribution of this source code is not allowed without the consent of the author
 */

#include "OperationManager.h"
#include <linux/limits.h>
#define WHISPER_CPP_PATH	"~/whisper.cpp/"


/*
 * Constructor
 * */
OperationManager::OperationManager()
{
	// Init variables
	flagOperationComplete = 0;
	flagAbortOperation = 0;
	flagTerminateOperation = 0;
}


/*
 * Destructor
 * */
OperationManager::~OperationManager()
{
}


/*
 * Open interfaces.
 */
int OperationManager::openInterfaces()
{
	int errorCode = 0;
	
	
	// this->mifVoCoR = new ModuleInterface(MODULE_ID_VOCOR, "VoCoR_Interface");
	
	this->mifLiCAS = new ModuleInterface(MODULE_ID_LiCAS, "LiCAS_Interface");
	this->mifLiCAS->openUDPSocket("192.168.0.166", 24001, 25001);	// Transmit through port 24001, Receive from port 25001
	
	this->mifVision = new ModuleInterface(MODULE_ID_VISION, "Vision_Interface");
	this->mifVision->openUDPSocket("192.168.0.166", 24003, 25003);
	
	// this->mifArduPilot = new ModuleInterface(MODULE_ID_ARDUPILOT, "ArduPilot_Interface");
	
	// this->mifPlanner = new ModuleInterface(MODULE_ID_PLANNER, "Planner_Interface");
	
	this->mifTarget = new ModuleInterface(MODULE_ID_TARGET, "Target_Interface");
	this->mifTarget->openUDPSocket("127.0.0.1", 24004, 25004);
	
	return errorCode;
}
	
/*
 * Get operation parameters (Supply Point, Delivery Point, packet ID)
 */
int OperationManager::getOperationParams()
{
	int errorCode = 0;
	int mission_defined = 0;
	
	std::ifstream fileOutWhisperCSV;
	std::string line;
	std::string systemCmd;
	std::string userCommand;
	std::string userResponse;
	std::string robotReply;
	std::string option;
	char cwd[PATH_MAX];
	
	
	do
	{
		cout << "\tPress [r] or [R] to record order. " << endl;
		cout << "\tPress [q] or [Q] to record order. " << endl;
		cout << endl;
		cout << "\t\tOption: ";
		
		getline(cin, option);
		
		if(option == "r" || option == "R")
		{
			cout << "\tRECORDING. Press [Ctrl]+[C] to finish recording." << endl;
			// Call arecord to record audio (voice) from microphone and store it as .wav file.
			// Audio should be recorded at 16 kHz rate, with S16_LE format (16 bits signed, little endian).
			system("arecord -r 16000 -f S16_LE ./UserRequest.wav");
			// Call whisper.cpp to get the text from the wav file, generating a csv file with the output text
			getcwd(cwd, sizeof(cwd));
			systemCmd = "cd " + string(WHISPER_CPP_PATH) + "&& ./main -np -ocsv -f " + string(cwd) + "/UserRequest.wav";
			system(systemCmd.c_str());
			// Open the generated CSV file
			fileOutWhisperCSV.open("UserRequest.wav.csv");
			if(fileOutWhisperCSV.is_open() == false)
				cout << "ERROR: could not open UserRequest.wav.csv file." << endl;
			else
			{
				// Read first line of the WhisperCpp CSV file (ignore)
				std::getline(fileOutWhisperCSV, line);
				// Get the user command
				std::getline(fileOutWhisperCSV, line);
				userCommand = line.substr(line.find("\"")+2, line.length() - (line.find("\"")+2) - 2);
				cout << "User command: [" << userCommand << "]" << endl;
				// Close file
				fileOutWhisperCSV.close();
                
                string cleanedUserCommand;
				for (char c : userCommand) {
					if (std::isalnum(c)) { // Check if the character is alphanumeric or a space
						cleanedUserCommand += std::tolower(c); // Convert to lowercase and append to the result
					}
				}

				// Search for keywords and extract parameters
                size_t parcelPos = cleanedUserCommand.find("parcel");
                size_t pickUpPos = cleanedUserCommand.find("pickuppoint");
                size_t deliveryPos = cleanedUserCommand.find("deliverypoint");

                // Extract Parcel ID
                for (size_t i = 0; i < cleanedUserCommand.length(); ++i)
                {
                    if (std::isdigit(cleanedUserCommand[i]))
                    {
                        this->parcelID = cleanedUserCommand[i] - '0'; // Convert char to integer
                        break;
                    }
                }
                
                // Extract PickUp Point
				if (pickUpPos != std::string::npos)
				{
					size_t start = pickUpPos + 11; // Position after "pickuppoint"
					while (std::isspace(cleanedUserCommand[start])) start++; // Skip whitespace
					char nextChar = cleanedUserCommand[start]; // Get the next character
					this->supplyPointName = std::string(1, std::toupper(nextChar)); // Convert to uppercase and store as string
				}

                // Extract Delivery Point
				if (deliveryPos != std::string::npos)
				{
					size_t start = deliveryPos + 13; // Position after "deliverypoint"
					while (std::isspace(cleanedUserCommand[start])) start++; // Skip whitespace
					char nextChar = cleanedUserCommand[start]; // Get the next character
					this->deliveryPointName = std::string(1, std::toupper(nextChar)); // Convert to uppercase and store as string
				}
                
				cout << "\tParcel ID:\t" << this->parcelID << endl;
				cout << "\tSupply Point:\t" << this->supplyPointName << endl;
				cout << "\tDelivery Point:\t" << this->deliveryPointName << endl;
				
				// Reproduce command by voice (TTS)
				robotReply = "you have requested to " + userCommand + ". Is this correct? Please, type yes or no.";
				systemCmd = "echo " + robotReply + " | festival --tts";
				system(systemCmd.c_str());
				
				cout << "Confirm [yes/no]: ";
				
				std::getline(cin, userResponse);
				if(userResponse == "no" || userResponse == "no" || userResponse == "NO")
				{
					robotReply = "Then I will discard this request.";
					systemCmd = "echo " + robotReply + " | festival --tts";
					system(systemCmd.c_str());
				}
				else
				{
					robotReply = "OK. Proceeding with the operation.";
					systemCmd = "echo " + robotReply + " | festival --tts";
					system(systemCmd.c_str());
					mission_defined = 1;
				}
			}
		}
		else if(option == "q" || option == "Q")
		{
				robotReply = "I am glad to work with you.";
				systemCmd = "echo " + robotReply + " | festival --tts";
				system(systemCmd.c_str());
		}
		else
			cout << "Invalid option. Try again." << endl;
			
	} while(option != "q" && option != "Q" && mission_defined == 0);
	
	return errorCode;
}
	
/*
 * Start operation
 */
int OperationManager::startOperation()
{
	DATA_PACKET_ROBOT_POSE TOLP = {0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0};	
	DATA_PACKET_ROBOT_POSE spG = {2.3, -1.74, 1.0, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE spF = {2.25, 1.38, 1.0, 0.0, 0.0, 0.0, 1.0};
	//DATA_PACKET_ROBOT_POSE dpE = {5.02, -0.23, 0.55, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE dpE = {4.94, -0.21, 1.2, 0.0, 0.0, 0.0, 1.0};	
	DATA_PACKET_ROBOT_POSE dpD = {5.24, 2.04, 0.65, 0.0, 0.0, 0.0, 1.0};

	DATA_PACKET_ROBOT_POSE spGhigh = {2.3, -1.74, 1.5, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE spFhigh = {2.25, 1.38, 1.5, 0.0, 0.0, 0.0, 1.0};
//	DATA_PACKET_ROBOT_POSE dpEhigh = {5.02, -0.23, 1.50, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE dpEhigh = {4.94, -0.21, 1.5, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE dpDhigh = {5.24, 2.04, 1.50, 0.0, 0.0, 0.0, 1.0};
	
	DATA_PACKET_ROBOT_POSE spGbackandhigh = {1.35, 1.38, 1.5, 0.0, 0.0, 0.0, 1.0};
	DATA_PACKET_ROBOT_POSE spFbackandhigh = {1.4, -1.74, 1.5, 0.0, 0.0, 0.0, 1.0};
	
	int msgCode_LiCAS = 0, msgOption_LiCAS = 0;
	int msgCode_Target = 0, msgOption_Target = 0;
	string cmd;
	int errorCode = 0;
	
	// Phase 0: Take-off from TOLP
	
	cout << "Type [start] to begin operation: ";
	cin >> cmd;
	
	cout << "Starting operation" << endl;		
	
	this->parcelID = 4;
	this->supplyPointName[0] = 'F';
	this->deliveryPointName[0] = 'E';
	
	// Phase 1: Testing the arms
	cout << "LiCAS Request: go to rest position...";
	this->mifLiCAS->sendModuleRequest(1, 0);
	while(this->mifLiCAS->isMessageReceived() == 0)
		usleep(10000);	// Wait 100 ms
	this->mifLiCAS->getMessage(msgCode_LiCAS, msgOption_LiCAS);
	if(msgCode_LiCAS == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	
	usleep(2000000);
	
	cout << "LiCAS Request: go to operation position..." << endl;
	this->mifLiCAS->sendModuleRequest(2, 0);
	while(this->mifLiCAS->isMessageReceived() == 0)
		usleep(100000);	// Wait 100 ms
	this->mifLiCAS->getMessage(msgCode_LiCAS, msgOption_LiCAS);
	if(msgCode_LiCAS == 2)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	
	usleep(2000000);
	
	// Phase 2: Approach to Supply Point
	cout << "Target Request: go to supply point..." << endl;
	switch(this->supplyPointName[0])
	{
		case 'G':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&spG, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << spG.pos[0] << "  " << spG.pos[1] << "  " << spG.pos[2]  << endl;
			cout << "quat:  " << spG.quat[0] << "  " << spG.quat[1] << "  " << spG.quat[2] << "  " << spG.quat[3] << endl;
			break;
		}
		case 'F':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&spF, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << spF.pos[0] << "  " << spF.pos[1] << "  " << spF.pos[2]  << endl;
			cout << "quat:  " << spF.quat[0] << "  " << spF.quat[1] << "  " << spF.quat[2] << "  " << spF.quat[3] << endl;
			break;
		}
	}
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	if(msgCode_Target == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	usleep(2000000);
	
	// Phase 3: Enable vision module and localize parcel
	cout << "Enabling vision module...";
	this->mifVision->sendModuleRequest(1, this->parcelID);
	cout << "DONE." << endl;
	
	usleep(1000000);
	
	// Phase 4: Positioning according to the parcel
	cout << "Drone enters in aruco guidance" << endl;
	this->mifTarget->sendModuleRequest(2, 0);
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	if(msgCode_Target == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	
	// Phase 5: Retrieve parcel
	cout << "Grasping parcel..." << endl;
	this->mifLiCAS->sendModuleRequest(3, 0);
	while(this->mifLiCAS->isMessageReceived() == 0)
		usleep(100000);	// Wait 100 ms
	this->mifLiCAS->getMessage(msgCode_LiCAS, msgOption_LiCAS);
	if(msgCode_LiCAS == 3)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	usleep(2000000);
	
	// Phase 6: Fly away from Delivery Point
	cout << "Target Request: go away from supply point..." << endl;
	mifTarget->sendModuleRequest(3, 0);
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	if(msgCode_Target == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	usleep(1000000);
		
	// Phase 7: Go to Delivery Point
	cout << "Target Request: go to delivery point...";
	switch(this->deliveryPointName[0])
	{
		case 'E':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpEhigh, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpEhigh.pos[0] << "  " << dpEhigh.pos[1] << "  " << dpEhigh.pos[2]  << endl;
			cout << "quat:  " << dpEhigh.quat[0] << "  " << dpEhigh.quat[1] << "  " << dpEhigh.quat[2] << "  " << dpEhigh.quat[3] << endl;
			break;
		}
		case 'D':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpDhigh, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpDhigh.pos[0] << "  " << dpDhigh.pos[1] << "  " << dpDhigh.pos[2]  << endl;
			cout << "quat:  " << dpDhigh.quat[0] << "  " << dpDhigh.quat[1] << "  " << dpDhigh.quat[2] << "  " << dpDhigh.quat[3] << endl;
			break;
		}
	}
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	
	switch(this->deliveryPointName[0])
	{
		case 'E':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpE, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpE.pos[0] << "  " << dpE.pos[1] << "  " << dpE.pos[2]  << endl;
			cout << "quat:  " << dpE.quat[0] << "  " << dpE.quat[1] << "  " << dpE.quat[2] << "  " << dpE.quat[3] << endl;
			break;
		}
		case 'D':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpD, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpD.pos[0] << "  " << dpD.pos[1] << "  " << dpD.pos[2]  << endl;
			cout << "quat:  " << dpD.quat[0] << "  " << dpD.quat[1] << "  " << dpD.quat[2] << "  " << dpD.quat[3] << endl;
			break;
		}
	}
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	if(msgCode_Target == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	usleep(6000000);
	
	// Phase 8: Drop parcel
	cout << "LiCAS Request: go to rest position..." << endl;
	this->mifLiCAS->sendModuleRequest(1, 0);
	while(this->mifLiCAS->isMessageReceived() == 0)
		usleep(10000);	// Wait 100 ms
	this->mifLiCAS->getMessage(msgCode_LiCAS, msgOption_LiCAS);
	if(msgCode_LiCAS == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	
	usleep(2000000);
	
	cout << "LiCAS Request: go to operation position..." << endl;
	this->mifLiCAS->sendModuleRequest(2, 0);
	while(this->mifLiCAS->isMessageReceived() == 0)
		usleep(100000);	// Wait 100 ms
	this->mifLiCAS->getMessage(msgCode_LiCAS, msgOption_LiCAS);
	if(msgCode_LiCAS == 2)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	
	usleep(2000000);
	
	// Phase 9: Go back to TOLP
	cout << "Target Request: go to TOLP point...";
	
	switch(this->deliveryPointName[0])
	{
		case 'E':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpEhigh, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpEhigh.pos[0] << "  " << dpEhigh.pos[1] << "  " << dpEhigh.pos[2]  << endl;
			cout << "quat:  " << dpEhigh.quat[0] << "  " << dpEhigh.quat[1] << "  " << dpEhigh.quat[2] << "  " << dpEhigh.quat[3] << endl;
			break;
		}
		case 'D':
		{
			mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&dpDhigh, sizeof(DATA_PACKET_ROBOT_POSE));
			cout << "pos:   " << dpDhigh.pos[0] << "  " << dpDhigh.pos[1] << "  " << dpDhigh.pos[2]  << endl;
			cout << "quat:  " << dpDhigh.quat[0] << "  " << dpDhigh.quat[1] << "  " << dpDhigh.quat[2] << "  " << dpDhigh.quat[3] << endl;
			break;
		}
	}
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	
	mifTarget->sendModuleRequestParams(1, 0, (uint8_t*)&TOLP, sizeof(DATA_PACKET_ROBOT_POSE));
		cout << "pos:   " << TOLP.pos[0] << "  " << TOLP.pos[1] << "  " << TOLP.pos[2]  << endl;
		cout << "quat:  " << TOLP.quat[0] << "  " << TOLP.quat[1] << "  " << TOLP.quat[2] << "  " << TOLP.quat[3] << endl;
	while(this->mifTarget->isMessageReceived() == 0) //waiting until the order is fully executed
		usleep(100000);	// Wait 100 ms
	this->mifTarget->getMessage(msgCode_Target, msgOption_Target);
	if(msgCode_Target == 1)
		cout << "DONE." << endl;
	else
		cout << "Request not correctly executed." << endl;
	usleep(2000000);
	
	return errorCode;
}
	
/*
 * Terminate operation.
 */
int OperationManager::terminateOperation()
{
	int errorCode = 0;
	
	
	
	
	return errorCode;
}
	
/*
 * Close interfaces.
 */
int OperationManager::closeInterfaces()
{
	int errorCode = 0;
	
	
	// this->mifVoCoR->closeUDPSocket();
	this->mifLiCAS->closeUDPSocket();
	this->mifVision->closeUDPSocket();
	// this->mifArduPilot->closeUDPSocket();
	// this->mifPlanner->closeUDPSocket();
	this->mifTarget->closeUDPSocket();
	
	
	
	return errorCode;
}


