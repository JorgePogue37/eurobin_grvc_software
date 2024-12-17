/*
 * Copyright (c) 2024 LiCAS Robotic Arms
 *
 * Developer: Alejandro Suarez Fernandez-Miranda
 *
 * This source code is part of LiCAS Robotic Arms.
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Constants.h
 */
 

#ifndef CONSTANTS_H_
#define CONSTANTS_H_



/******************************* Enabled Devices/Messages *******************************/

#define ARMS_CONTROL_ENABLED				1
#define ARMS_FEEDBACK_ENABLED				1
#define STM32BOARD_AVAILABLE				0

#define LEFT_ARM_WARNING_DISPLAY_FLAG		1
#define RIGHT_ARM_WARNING_DISPLAY_FLAG		1
#define STM32BOARD_WARNING_DISPLAY_FLAG		0


/******************************* ID's *******************************/

// Arms ID's
#define LEFT_ARM_ID				1
#define RIGHT_ARM_ID			2

// Servos ID's
#define LEFT_ARM_SHOULDER_PITCH_SERVO_ID	1		// Herkulex DRS-0602
#define LEFT_ARM_SHOULDER_ROLL_SERVO_ID		2		// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_YAW_SERVO_ID		3		// Herkulex DRS-0402
#define LEFT_ARM_ELBOW_PITCH_SERVO_ID		4		// Herkulex DRS-0402
#define LEFT_ARM_WRIST_ROLL_SERVO_ID		5		// Herkulex DRS-0101
#define LEFT_ARM_WRIST_PITCH_SERVO_ID		6		// Herkulex DRS-0101
#define LEFT_ARM_WRIST_YAW_SERVO_ID			7		// Herkulex DRS-0101

#define RIGHT_ARM_SHOULDER_PITCH_SERVO_ID	1		// Herkulex DRS-0602
#define RIGHT_ARM_SHOULDER_ROLL_SERVO_ID	2		// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_YAW_SERVO_ID		3		// Herkulex DRS-0402
#define RIGHT_ARM_ELBOW_PITCH_SERVO_ID		4		// Herkulex DRS-0402
#define RIGHT_ARM_WRIST_ROLL_SERVO_ID		5		// Herkulex DRS-0101
#define RIGHT_ARM_WRIST_PITCH_SERVO_ID		6		// Herkulex DRS-0101
#define RIGHT_ARM_WRIST_YAW_SERVO_ID		7		// Herkulex DRS-0101



/******************************* IP addresses, UDP ports *******************************/

#define GCS_UDP_PORT				22000			// Ground Control Station UDP port (for receiving commands/keep alive message)
#define TELEOPERATION_UDP_PORT		22001
#define VISION_SENSOR_UDP_PORT		42000



/******************************* Cartesian offset constants *******************************/

// Displacement of the coordinate system associated to the left arm with respect to the global frame (in meters)
#define LEFT_ARM_X_AXIS_OFFSET			0.0F
#define LEFT_ARM_Y_AXIS_OFFSET			0.1725F
#define LEFT_ARM_Z_AXIS_OFFSET			0.0F
// Displacement of the coordinate system associated to the right arm with respect to the global frame (in meters)
#define RIGHT_ARM_X_AXIS_OFFSET			0.0F
#define RIGHT_ARM_Y_AXIS_OFFSET			-0.1725F
#define RIGHT_ARM_Z_AXIS_OFFSET			0.0F




/******************************* Arm kinematics constants *******************************/

// Arm lengths
#define L1					0.3F	// Upper arm length
#define L2					0.3F	// Forearm length
#define IK_SOLUTION_ERROR_LIMIT			0.005F	// Maximum error allowed between the direct-inverse kinematics solution



/******************************* Camera offset and orientation constants *******************************/

// Displacement of the coordinate system associated to the camera with respect to the global frame (in meters)
#define CAMERA_OFFSET_X				-0.3F
#define CAMERA_OFFSET_Y				0.00F
#define CAMERA_OFFSET_Z				-0.08F

// Orientation of the camera with respect to the global frame (Euler ZYX, in degrees)
#define CAMERA_ORIENTATION_YAW			0.0F
#define CAMERA_ORIENTATION_PITCH		30.0F
#define CAMERA_ORIENTATION_ROLL			0.0F

#define VISION_CORRECTION_X_AXIS		1.0F
#define VISION_CORRECTION_Z_AXIS		1.0F


/******************************* Servo rotation limits, offsets and sign criteria *******************************/

// Number of servos per arm
#define NUM_ARM_JOINTS				4
#define NUM_ARM_SERVOS				4

// Left arm servo rotation limits (in degrees, physical limits, referred to the servo shaft frame)
#define LEFT_ARM_SHOULDER_PITCH_MAX		120	// Herkulex DRS-0602
#define LEFT_ARM_SHOULDER_PITCH_MIN		-120	// Herkulex DRS-0602
#define LEFT_ARM_SHOULDER_ROLL_MAX		120	// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_ROLL_MIN		-120	// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_YAW_MAX		90	// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_YAW_MIN		-90	// Herkulex DRS-0402
#define LEFT_ARM_ELBOW_PITCH_MAX		145	// Herkulex DRS-0402
#define LEFT_ARM_ELBOW_PITCH_MIN		-145	// Herkulex DRS-0402
// Right arm servo rotation limits (in degrees, physical limits, referred to the servo shaft frame)
#define RIGHT_ARM_SHOULDER_PITCH_MAX		120	// Herkulex DRS-0602
#define RIGHT_ARM_SHOULDER_PITCH_MIN		-120	// Herkulex DRS-0602
#define RIGHT_ARM_SHOULDER_ROLL_MAX		120	// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_ROLL_MIN		-120	// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_YAW_MAX		90	// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_YAW_MIN		-90	// Herkulex DRS-0402
#define RIGHT_ARM_ELBOW_PITCH_MAX		130	// Herkulex DRS-0402
#define RIGHT_ARM_ELBOW_PITCH_MIN		-130	// Herkulex DRS-0402


/*
 * CALIBRATION VALUES FOR LiCAS A1 - SN5
 */
// Left arm servo rotation offsets (in degrees, referred to the servo shaft frame)
#define LEFT_ARM_SHOULDER_PITCH_OFFSET	0.0F	// Herkulex DRS-0602
#define LEFT_ARM_SHOULDER_ROLL_OFFSET	0.0F	// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_YAW_OFFSET	0.0F	// Herkulex DRS-0402
#define LEFT_ARM_ELBOW_PITCH_OFFSET	0.0F	// Herkulex DRS-0402
// Right arm servo rotation offsets (in degrees, referred to the servo shaft frame)
#define RIGHT_ARM_SHOULDER_PITCH_OFFSET	0.0F	// Herkulex DRS-0602
#define RIGHT_ARM_SHOULDER_ROLL_OFFSET	0.0F	// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_YAW_OFFSET	0.0F	// Herkulex DRS-0402
#define RIGHT_ARM_ELBOW_PITCH_OFFSET	0.0F	// Herkulex DRS-0402



// Left arm servo rotation conversion sign (right arm criteria to servo shaft frame)
#define LEFT_ARM_SHOULDER_PITCH_SIGN	1	// Herkulex DRS-0602
#define LEFT_ARM_SHOULDER_ROLL_SIGN	-1	// Herkulex DRS-0402
#define LEFT_ARM_SHOULDER_YAW_SIGN	-1	// Herkulex DRS-0402
#define LEFT_ARM_ELBOW_PITCH_SIGN	-1	// Herkulex DRS-0402
// Right arm servo rotation offsets (right arm criteria to servo shaft frame)
#define RIGHT_ARM_SHOULDER_PITCH_SIGN	-1	// Herkulex DRS-0602
#define RIGHT_ARM_SHOULDER_ROLL_SIGN	-1	// Herkulex DRS-0402
#define RIGHT_ARM_SHOULDER_YAW_SIGN	-1	// Herkulex DRS-0402
#define RIGHT_ARM_ELBOW_PITCH_SIGN	1	// Herkulex DRS-0402


/******************************* Control constants *******************************/

// Time constants
#define SERVO_MOTION_CONTROL_PERIOD	0.01	// Elapsed time between two consecutive IJOG/SJOG data packets
#define SERVO_RAM_READ_INTERVAL		0.005	// Elapsed time between two consecutive RAM read request packets
#define DATA_LOG_PERIOD			0.02F

#define VISUAL_SERVOING_CONTROL_PERIOD	0.1F
#define VISUAL_SERVOING_CARTESIAN_SPEED	0.05F

#define EXTERNAL_CONTROL_REF_PERIOD	0.02


/******************************* Teleoperation constants *******************************/

#define TELEOPERATION_CONTROL_PERIOD	0.1F
#define MAX_3DCONNEXION_OFFSET_VALUE	0.684F
#define MAX_3DCONNEXION_ROTATION_VALUE	0.684F
#define SPEED_OFFSET_3DCONNEXION	0.4F
#define SPEED_ORIENTATION_3DCONNEXION	0.05F
#define THRESHOLD_3D_CONNEXION_OFFSET	0.075F
#define THRESHOLD_3D_CONNEXION_ROTATION	0.1F



/******************************* Communications constants *******************************/

// Data packet length
#define SERVO_DATA_PACKET_LENGTH	27
#define RX_BUFFER_LENGTH			256



/******************************* Command constants *******************************/
// Ground Control Station operation codes
#define GCS_CODE_NOP						0
#define GCS_CODE_GO_TO_REST_POSITION		1
#define GCS_CODE_GO_TO_OPERATION_POSITION	2
#define GCS_CODE_VISUAL_SERVOING			3
#define GCS_CODE_TELEOPERATION				4
#define GCS_CODE_TELEOPERATION_ARMS_GRASP	5
#define GCS_CODE_OPEN_LEFT_GRIPPER			6
#define GCS_CODE_CLOSE_LEFT_GRIPPER			66
#define GCS_CODE_OPEN_RIGHT_GRIPPER			7
#define GCS_CODE_CLOSE_RIGHT_GRIPPER		77
#define GCS_CODE_OPEN_BOTH_GRIPPERS			8
#define GCS_CODE_CLOSE_BOTH_GRIPPERS		88
#define GCS_CODE_SERVOS_CALIBRATION			10
#define GCS_CODE_JOINTS_IDENTIFICATION		11
#define GCS_CODE_GRASP_OBJECT				12
#define GCS_CODE_RELEASE_OBJECT				13

#define GCS_CODE_SEQUENCE_ROTATIONS			20


#define GCS_CODE_DISABLE_TORQUE_CONTROL		49
#define GCS_CODE_ENABLE_TORQUE_CONTROL		50

#define EXTERNAL_JOINT_CONTROL_REF			101
#define EXTERNAL_JOINT_CONTROL_SENDER		102
#define GCS_CODE_TRAJECTORY_CONTROL_IJ		131

// Gripper constants
#define GRIPPER_CMD_OPEN			1
#define GRIPPER_CMD_CLOSE			2
#define GRIPPER_CMD_SWITCH			3
#define GRIPPER_CMD_NOP				0

#define GRIPPER_STATE_RELEASED			0
#define GRIPPER_STATE_GRASPING			1






/******************************* Control interface *******************************/


#define ARMS_STATE_PUBLISHER_PERIOD	0.02

#define STATE_PUBLISHER_IP_ADDRESS	"127.0.0.1"
#define STATE_PUBLISHER_UDP_PORT	26001

#define CONTROL_REFERENCES_UDP_PORT	22008



#endif


