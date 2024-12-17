ADROM: Aerial Delivery Robot Operations Manager

This software module is intended to manage the realization of parcel grasping and delivery operations using an aerial robot with on-board mapping-localization, perception, and manipulation.

The operation consists of the following phases:

	(1) Take-off from Take-Off and Landing Position (TOLP, fixed position)
	(2) Approach to supply point (SP, coordinates as input parameter)
	(3) Correct position relative to parcel to be grasped based on vision
	(4) Grasp parcel with manipulator (parcel ID input parameter)
	(5) Go to delivery point (DP, coordinates as input parameter)
	(6) Drop parcel
	(7) Go back to TOLP
	
Input parameters are:

	- The XYZ-yaw coordinates of the supply point
	- The XYZ-yaw coordinates of the delivery point
	- The ID of the parcel to be grasped

The involved modules managed the the ADROM are:

	* The Voice Command for Robot (VoCoR) interface, used to obtain the input parameters from the user voice instruction.
	* The Multi-Rotor Path Planner (MRPP) that generates the trajectory from TOLP to SP, from SP to DP, and from DP to TOLP.
	* The Multi-Rotor Flight Manager (MRFM) that receives the input trajectory according to the operation phase.
	* The LiCAS Control Program (LCP) that conducts the parcel grasping from the SP and drop on the DP.
	* The Vision Module (VM) that provides the grasping points for the arms and position corrections for the multi-rotor.

Communications between the ADROM and the software modules will be implemented through a message service with UDP sockets.
Messages correspond to C-style structures representing a data packet sent through the UDP socket.

typedef struct
{
	uint8_t header[3];		// "MSG" character sequence
	uint16_t moduleID;		// See Table 1
	uint16_t requestCode;	// Code to be executed by a module
	float refPosition[3];	// Reference position XYZ in global frame (in [m])
	float refAttitude[3];	// Refference attitude in roll-pitch-yaw in global frame (in [m])
}

Table 1. IDs of the software modules

ID	|	Software Module
_______________________
0		ADROM
1		Voice Command for Robot (
2		LiCAS Control Program
3		Vision Module
4		Multi-rotor Flight Manager


