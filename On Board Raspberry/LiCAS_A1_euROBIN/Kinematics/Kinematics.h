/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Kinematics.h
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_


// Standard library
#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>

// #include <eigen3/Eigen/Dense>


// Specific library


// Constant definition
#define L1							0.31F
#define L2							0.26F
#define IK_SOLUTION_ERROR_LIMIT		0.005F
#define PI							3.14159265
#define NUM_ITERATIONS				25
#define N							50


using namespace std;
// using Eigen::MatrixXd;


class Kinematics
{
public:

	/***************** PUBLIC VARIABLES *****************/
	
	// MatrixXd J_TCP;
	// MatrixXd J_TCP_pinv;
	
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 * */
	Kinematics(uint8_t _armID, double _armXOffset, double _armYOffset, double _armZOffset);


	/*
	 * Destructor
	 * */
	virtual ~Kinematics();
	
	
	/*
	 * Computes the joint positions (q1: shoulder pitch, q2: shoulder roll, q3: shoulder yaw, q4: elbow pitch)
	 * for a given XYZ position of the wrist point/end-effector point.
	 * */
	int IKSolver(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi);
	int IKSolver_Q1Approx(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi, double q1_approx);
	int IKSolver_Q1Approx_LinksLength(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi, double q1_approx, double l1, double l2);
	int IKSolver_Decisor(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi);


	/*
	 * Computes the Cartesian position of the wrist point for the joint variables
	 */
	void directKinematics(double q1, double q2, double q3, double q4, double * x, double * y, double * z);
	void directKinematics_LinksLength(double q1, double q2, double q3, double q4, double * x, double * y, double * z, double l1, double l2);
	void directKinematics_Marker(double q1, double q2, double q3, double q4, double * xyzMarkerPosition);


	/*
	 * Update the Jacobian matrix of the TCP (3-DOF shoulder + 1-DOF elbow) and its pseudoinverse
	 */
	void updateJacobianTCP(double q1, double q2, double q3, double q4);


	/*
	 * Compute joint speed from Cartesian speed
	 */
	void computeJointSpeedInverseJacobian(double vx, double vy, double vz, double * dq1, double * dq2, double * dq3, double * dq4);

	/*
	 * Apply the direct kinematic model for checking if the inverse kinematics solution matches the
	 * specified XYZ position with an acceptable error.
	 * */
	int checkSolution(double x, double y, double z, double q1, double q2, double q3, double q4);
	int checkSolution_LinksLength(double x, double y, double z, double q1, double q2, double q3, double q4, double l1, double l2);
	
	

private:
	/***************** PRIVATE VARIABLES *****************/

	uint8_t armID;
	
	
	/***************** PRIVATE METHODS *****************/
	
	
	/*
	 * Look for the closest value of q1 such that  the corresponding evaluated function is closest to zero
	 */
	int lookForQ1Value(double x, double y, double z, double * q1, double q2);
	int lookForQ1ValueApprox(double x, double y, double z, double * q1, double q2, double q1_approx);
	int lookForQ1Solutions(double x, double y, double z, vector<double> & q1_solutions, double q2);


	/*
	 * Multiply 4x4 transformation matrices
	 */
	void multiplyTransformationMatrices(double M1[4][4], double M2[4][4], double resultMatrix[4][4]);
	
	
	/*
	 * Multiply a transformation matrix by a 4x1 vector
	 **/
	void multiplyTransformationMatrixVector(double M[4][4], double * V, double * resultVector);

	/*
	 * Returns 1 if value > 0, -1 if value < 0 and 0 if value == 0
	 * */
	int signDouble(double value);

};

#endif /* SERVOSTATE_H_ */


