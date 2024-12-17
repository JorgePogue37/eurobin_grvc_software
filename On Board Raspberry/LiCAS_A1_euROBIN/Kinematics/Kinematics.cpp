/*
 * Copyright (c) 2020-2024 Alejandro Suarez, asuarezfm@us.es
 *
 * This source code is part of the LiCAS Robotic Arms initiative, https://licas-robotic-arms.com/
 *
 * The distribution of this source code is not allowed without the consent of the author
 *
 * File name: Kinematics.cpp
 */

#include "Kinematics.h"



/*
 * Constructor
 * */
Kinematics::Kinematics(uint8_t _armID, double _armXOffset, double _armYOffset, double _armZOffset)
{
	armID = _armID;
	
	// Init matrices
	/*
	J_TCP = MatrixXd(3, 4);
	J_TCP_pinv = MatrixXd(4, 3);
	
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			J_TCP(i, j) = 0;
			J_TCP_pinv(j, i) = 0;
		}
	}
	*/
}


/*
 * Destructor
 * */
Kinematics::~Kinematics()
{
}
	
	
int Kinematics::IKSolver(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi)
{
	struct timeval tini;
	struct timeval tend;
	double argACos = 0;
	double elapsedTime = 0;
	int error = 0;
	
	
	// Initial time stamp
	gettimeofday(&tini, NULL);
	
	// Check if the solution is within the or in the origin
	if(sqrt(x*x + y*y + z*z) >= 0.95*(L1 + L2))
	{
		error = 1;
		printf("ERROR: [in Kinematics::IKSolver] non-reachable point for arm %d\n", armID);
	}
	else
	{
		// (1) Shoulder roll joint angle q2
		*q2 = phi;

		// (2) Elbow pitch joint angle q4
		argACos = (x*x + y*y + z*z - L1*L1 - L2*L2)/(2*L1*L2);
		if(argACos < -0.999 || argACos > 0.999)
		{
			error = 1;
			printf("ERROR: [in Kinematics::IKSolver] invalid cosine argument for the elbow pitch joint on arm %d\n", armID);
		}
		else
			*q4 = -acos(argACos);
	
		// (3) Shoulder pitch angle q1 (numeric method)
		if(error == 0)
			error = lookForQ1Value(x, y, z, q1, *q2);
		
		// (4) Shoulder yaw angle (q3)
		if(error == 0)
		{
			double xw2 = x*cos(*q1) - z*sin(*q1);
			double yw2 = x*sin(*q1)*sin(*q2) + y*cos(*q2) + z*cos(*q1)*sin(*q2);
			*q3 = atan2(yw2, xw2);
		}
		
		// Check if solution is correct
		if(error == 0)	
			checkSolution(x, y, z, *q1, *q2, *q3, *q4);
			
		// Final time stamp
		gettimeofday(&tend, NULL);
		elapsedTime = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini .tv_usec);
		// printf("IK solver computational time: %.2lf [ms]\n", 1e3*elapsedTime);
	}
	
	
	return error;
}
	
	
int Kinematics::IKSolver_Q1Approx(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi, double q1_approx)
{
	struct timeval tini;
	struct timeval tend;
	double argACos = 0;
	double elapsedTime = 0;
	int error = 0;
	
	
	// Initial time stamp
	gettimeofday(&tini, NULL);
	
	// Check if the solution is within the or in the origin
	if(sqrt(x*x + y*y + z*z) >= 0.95*(L1 + L2))
	{
		error = 1;
		printf("ERROR: [in Kinematics::IKSolver] non-reachable point for arm %d\n", armID);
	}
	else
	{
		// (1) Shoulder roll joint angle q2
		*q2 = phi;

		// (2) Elbow pitch joint angle q4
		argACos = (x*x + y*y + z*z - L1*L1 - L2*L2)/(2*L1*L2);
		if(argACos < -0.999 || argACos > 0.999)
		{
			error = 1;
			printf("ERROR: [in Kinematics::IKSolver] invalid cosine argument for the elbow pitch joint on arm %d\n", armID);
		}
		else
			*q4 = -acos(argACos);
	
		// (3) Shoulder pitch angle q1 (numeric method)
		if(error == 0)
		{
			error = lookForQ1ValueApprox(x, y, z, q1, *q2, q1_approx);
		}
		
		// (4) Shoulder yaw angle (q3)
		if(error == 0)
		{
			double xw2 = x*cos(*q1) - z*sin(*q1);
			double yw2 = x*sin(*q1)*sin(*q2) + y*cos(*q2) + z*cos(*q1)*sin(*q2);
			*q3 = atan2(yw2, xw2);
			if(*q3 >= 175*PI/180 || *q3 <= -175*PI/180)
				*q3 = 0;
		}
		
		// Check if solution is correct
		if(error == 0)	
			checkSolution(x, y, z, *q1, *q2, *q3, *q4);
			
		// Final time stamp
		gettimeofday(&tend, NULL);
		elapsedTime = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini .tv_usec);
		// printf("IK solver computational time: %.2lf [ms]\n", 1e3*elapsedTime);
	}
	
	
	return error;
}
	
	
int Kinematics::IKSolver_Q1Approx_LinksLength(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi, double q1_approx, double l1, double l2)
{
	struct timeval tini;
	struct timeval tend;
	double argACos = 0;
	double elapsedTime = 0;
	int error = 0;
	
	
	// Initial time stamp
	gettimeofday(&tini, NULL);
	
	// Check if the solution is within the or in the origin
	if(sqrt(x*x + y*y + z*z) >= 0.95*(l1 + l2))
	{
		error = 1;
		printf("ERROR: [in Kinematics::IKSolver] non-reachable point for arm %d\n", armID);
	}
	else
	{
		// (1) Shoulder roll joint angle q2
		*q2 = phi;

		// (2) Elbow pitch joint angle q4
		argACos = (x*x + y*y + z*z - l1*l1 - l2*l2)/(2*l1*l2);
		if(argACos < -0.999 || argACos > 0.999)
		{
			error = 1;
			printf("ERROR: [in Kinematics::IKSolver] invalid cosine argument for the elbow pitch joint on arm %d\n", armID);
		}
		else
			*q4 = -acos(argACos);
	
		// (3) Shoulder pitch angle q1 (numeric method)
		if(error == 0)
		{
			error = lookForQ1ValueApprox(x, y, z, q1, *q2, q1_approx);
		}
		
		// (4) Shoulder yaw angle (q3)
		if(error == 0)
		{
			double xw2 = x*cos(*q1) - z*sin(*q1);
			double yw2 = x*sin(*q1)*sin(*q2) + y*cos(*q2) + z*cos(*q1)*sin(*q2);
			*q3 = atan2(yw2, xw2);
			if(*q3 >= 175*PI/180 || *q3 <= -175*PI/180)
				*q3 = 0;
		}
		
		// Check if solution is correct
		if(error == 0)	
			checkSolution_LinksLength(x, y, z, *q1, *q2, *q3, *q4, l1, l2);
		
		// Final time stamp
		gettimeofday(&tend, NULL);
		elapsedTime = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini .tv_usec);
		// printf("IK solver computational time: %.2lf [ms]\n", 1e3*elapsedTime);
	}
	
	
	return error;
}


/*
 * Computes the Cartesian position of the wrist point for the joint variables.
 */
void Kinematics::directKinematics(double q1, double q2, double q3, double q4, double * x, double * y, double * z)
{
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double resultMatrix1[4][4];
	double resultMatrix2[4][4];
	double resultMatrix3[4][4];
	double p4_wrist[4];
	double p0_wrist[4];
	double x_ = 0;
	double y_ = 0;
	double z_ = 0;
	double ex = 0;
	double ey = 0;
	double ez = 0;
	double e = 0;	
	int error = 0;
	
	
	// Initialize matrices
	for(unsigned int row = 0; row < 4; row++)
	{
		for(unsigned int col = 0; col < 4; col++)
		{
			if(row != col)
			{
				T01[row][col] = 0;
				T12[row][col] = 0;
				T23[row][col] = 0;
				T34[row][col] = 0;
			}
			else
			{
				T01[row][col] = 1.0;
				T12[row][col] = 1.0;
				T23[row][col] = 1.0;
				T34[row][col] = 1.0;
			}
		}
	}
	
	// Set the value of the shoulder pitch transformation matrix
	T01[0][0] = cos(q1);
	T01[0][2] = sin(q1);
	T01[2][0] = -sin(q1);
	T01[2][2] = cos(q1);
	// Set the value of the shoulder roll transformation matrix
	T12[1][1] = cos(q2);
	T12[1][2] = -sin(q2);
	T12[2][1] = sin(q2);
	T12[2][2] = cos(q2);
	// Set the value of the shoulder yaw transformation matrix
	T23[0][0] = cos(q3);
	T23[0][1] = -sin(q3);
	T23[1][0] = sin(q3);
	T23[1][1] = cos(q3);
	// Set the value of the elbow pitch transformation matrix
	T34[0][0] = cos(q4);
	T34[0][2] = sin(q4);
	T34[2][0] = -sin(q4);
	T34[2][2] = cos(q4);
	T34[2][3] = -L1;
	// Set the value of the wrist point referred to the elbow pitch frame
	p4_wrist[0] = 0;
	p4_wrist[1] = 0;
	p4_wrist[2] = -L2;
	p4_wrist[3] = 1;
	
	// Compute the value of the wrist point referred to the shoulder pitch frame
	multiplyTransformationMatrices(T01, T12, resultMatrix1);
	multiplyTransformationMatrices(resultMatrix1, T23, resultMatrix2);
	multiplyTransformationMatrices(resultMatrix2, T34, resultMatrix3);
	multiplyTransformationMatrixVector(resultMatrix3, p4_wrist, p0_wrist);

	*x = p0_wrist[0];
	*y = p0_wrist[1];
	*z = p0_wrist[2];
}


/*
 * Computes the Cartesian position of the wrist point for the joint variables.
 */
void Kinematics::directKinematics_LinksLength(double q1, double q2, double q3, double q4, double * x, double * y, double * z, double l1, double l2)
{
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double resultMatrix1[4][4];
	double resultMatrix2[4][4];
	double resultMatrix3[4][4];
	double p4_wrist[4];
	double p0_wrist[4];
	double x_ = 0;
	double y_ = 0;
	double z_ = 0;
	double ex = 0;
	double ey = 0;
	double ez = 0;
	double e = 0;	
	int error = 0;
	
	
	// Initialize matrices
	for(unsigned int row = 0; row < 4; row++)
	{
		for(unsigned int col = 0; col < 4; col++)
		{
			if(row != col)
			{
				T01[row][col] = 0;
				T12[row][col] = 0;
				T23[row][col] = 0;
				T34[row][col] = 0;
			}
			else
			{
				T01[row][col] = 1.0;
				T12[row][col] = 1.0;
				T23[row][col] = 1.0;
				T34[row][col] = 1.0;
			}
		}
	}
	
	// Set the value of the shoulder pitch transformation matrix
	T01[0][0] = cos(q1);
	T01[0][2] = sin(q1);
	T01[2][0] = -sin(q1);
	T01[2][2] = cos(q1);
	// Set the value of the shoulder roll transformation matrix
	T12[1][1] = cos(q2);
	T12[1][2] = -sin(q2);
	T12[2][1] = sin(q2);
	T12[2][2] = cos(q2);
	// Set the value of the shoulder yaw transformation matrix
	T23[0][0] = cos(q3);
	T23[0][1] = -sin(q3);
	T23[1][0] = sin(q3);
	T23[1][1] = cos(q3);
	// Set the value of the elbow pitch transformation matrix
	T34[0][0] = cos(q4);
	T34[0][2] = sin(q4);
	T34[2][0] = -sin(q4);
	T34[2][2] = cos(q4);
	T34[2][3] = -l1;
	// Set the value of the wrist point referred to the elbow pitch frame
	p4_wrist[0] = 0;
	p4_wrist[1] = 0;
	p4_wrist[2] = -l2;
	p4_wrist[3] = 1;
	
	// Compute the value of the wrist point referred to the shoulder pitch frame
	multiplyTransformationMatrices(T01, T12, resultMatrix1);
	multiplyTransformationMatrices(resultMatrix1, T23, resultMatrix2);
	multiplyTransformationMatrices(resultMatrix2, T34, resultMatrix3);
	multiplyTransformationMatrixVector(resultMatrix3, p4_wrist, p0_wrist);

	*x = p0_wrist[0];
	*y = p0_wrist[1];
	*z = p0_wrist[2];
}





/*
 * Update the Jacobian matrix of the TCP (3-DOF shoulder + 1-DOF elbow) and its pseudoinverse
 */
void Kinematics::updateJacobianTCP(double q1, double q2, double q3, double q4)
{
	/*
	J_TCP(0,0) = L2*(sin(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4)) - L1*cos(q1)*cos(q2);
	J_TCP(0,1) = L2*(cos(q4)*sin(q1)*sin(q2) - cos(q2)*sin(q1)*sin(q3)*sin(q4)) + L1*sin(q1)*sin(q2);
	J_TCP(0,2) = L2*sin(q4)*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2));
	J_TCP(0,3) = -L2*(cos(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - cos(q2)*sin(q1)*sin(q4));

	J_TCP(1,0) = 0;
	J_TCP(1,1) = L2*(cos(q2)*cos(q4) + sin(q2)*sin(q3)*sin(q4)) + L1*cos(q2);
	J_TCP(1,2) = -L2*cos(q2)*cos(q3)*sin(q4);
	J_TCP(1,3) = -L2*(sin(q2)*sin(q4) + cos(q2)*cos(q4)*sin(q3));

	J_TCP(2,0) = L2*(sin(q4)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1)) + L1*cos(q2)*sin(q1);
	J_TCP(2,1) = L2*(cos(q1)*cos(q4)*sin(q2) - cos(q1)*cos(q2)*sin(q3)*sin(q4)) + L1*cos(q1)*sin(q2);
	J_TCP(2,2) = -L2*sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2));
	J_TCP(2,3) = L2*(cos(q4)*(cos(q3)*sin(q1) - cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4));
	
	
	J_TCP_pinv = J_TCP.completeOrthogonalDecomposition().pseudoInverse();
	*/
}


/*
 * Compute joint speed from Cartesian speed
 */
void Kinematics::computeJointSpeedInverseJacobian(double vx, double vy, double vz, double * dq1, double * dq2, double * dq3, double * dq4)
{
	/*
	Eigen::Vector3d v(vx, vy, vz);
	Eigen::Vector4d dq(0, 0, 0, 0);
	
	
	dq = J_TCP_pinv * v;
	
	*dq1 = dq(0);
	*dq2 = dq(1);
	*dq3 = dq(2);
	*dq4 = dq(3);
	*/
}


	
/*
 * Look for the closest value of q1 such that the corresponding evaluated function is closest to zero
 */
int Kinematics::lookForQ1Value(double x, double y, double z, double * _q1, double q2)
{
	double q1 = 0;
	double vk = 0;
	double vk_1L = 0;
	double vk_1R = 0;
	double w = 0;
	const double q1_step = 0.0078;
	const double epsilon_v = 1e-3;
	bool solutionFoundFlag = false;
	int error = 0;
	
	
	w = (L2*L2 - (L1*L1 + x*x + y*y + z*z) + 2*L1*y*sin(q2))/(2*L1*cos(q2));
	
	// Check if q1 = 0 is a solution
	vk = x*sin(0) + z*cos(0) - w;
	vk_1L = vk;
	vk_1R = vk;
	if(fabs(vk) < epsilon_v)
	{
		*_q1 = 0;
		solutionFoundFlag = true;
	}
	else
	{
		// Look for the value of q1 such that v = 0 and it is closest to zero (q1 > 0 || q1 < 0)
		for(q1 = q1_step; q1 < 0.95*1.5708 && solutionFoundFlag == false; q1 += q1_step)
		{
			// Check the zeros cross of v for positive values of q1
			vk = x*sin(q1) + z*cos(q1) - w;
			if(fabs(vk) < epsilon_v || (vk_1R < 0 && vk > 0) || (vk_1R > 0 && vk < 0))
			{
				*_q1 = q1;
				solutionFoundFlag = true;
			}
			else
			{
				vk_1R = vk;
				
				// Check the zero cross of v for negative values of q1
				vk = x*sin(-q1) + z*cos(-q1) - w;
				if(fabs(vk) < epsilon_v || (vk_1L < 0 && vk > 0) || (vk_1L > 0 && vk < 0))
				{
					*_q1 = -q1;
					solutionFoundFlag = true;
				}
				else
					vk_1L = vk;
			}
		}
	}
	
	if(solutionFoundFlag == false)
	{
		printf("ERROR: [in Kinematics::lookForQ1Value]: no solution could be found for arm %d\n", armID);
		error = 1;
	}
	
	
	return error;
}


/*
 * Look for the closest value of q1 such that the corresponding evaluated function is closest to q1_approx
 */
int Kinematics::lookForQ1ValueApprox(double x, double y, double z, double * _q1, double q2, double q1_approx)
{
	double q1 = 0;
	double vk = 0;
	double vk_1 = 0;
	double w = 0;
	const double q1_step = 0.015708;
	const double epsilon_v = 1e-3;
	bool solutionFoundFlag = false;
	int error = 0;
	
	
	w = (L2*L2 - (L1*L1 + x*x + y*y + z*z) + 2*L1*y*sin(q2))/(2*L1*cos(q2));
	
	// Look for a positive value of q1 such that there is a change in the sign of v
	for(q1 = -0.9*1.5708; q1 < 0.9*1.5708; q1 += q1_step)
	{
		// Check the zero cross of v
		vk = x*sin(q1) + z*cos(q1) - w;
		vk_1 = x*sin(q1 - q1_step) + z*cos(q1 - q1_step) - w;
		if(fabs(vk) < epsilon_v || (vk_1 < 0 && vk > 0) || (vk_1 > 0 && vk < 0))
		{
			if(solutionFoundFlag == false)
			{
				*_q1 = q1;
				solutionFoundFlag = true;
			}
			else
			{
				// Check if the new solution is closest to q1_approx
				if(fabs(*_q1 - q1_approx) > fabs(q1 - q1_approx))
					*_q1 = q1;
			}
		}
	}
	
	if(solutionFoundFlag == false)
	{
		printf("ERROR: [in Kinematics::lookForQ1Value]: no solution could be found for arm %d\n", armID);
		error = 1;
	}
	
	
	return error;
}


int Kinematics::IKSolver_Decisor(double * q1, double * q2, double * q3, double * q4, double x, double y, double z, double phi)
{
	struct timeval tini;
	struct timeval tend;
	double argACos = 0;
	double elapsedTime = 0;
	vector <double> q1_solutions;
	double q1_aux = 0;
	double q3_aux = 0;
	int error = 0;
	
	
	// Initial time stamp
	gettimeofday(&tini, NULL);
	
	// Check if the solution is within the volume of operation
	if(sqrt(x*x + y*y + z*z) >= 0.95*(L1 + L2))
	{
		error = 1;
		printf("ERROR: [in Kinematics::IKSolver] non-reachable point for arm %d\n", armID);
	}
	else
	{
		// (1) Shoulder roll joint angle q2
		*q2 = phi;

		// (2) Elbow pitch joint angle q4
		argACos = (x*x + y*y + z*z - L1*L1 - L2*L2)/(2*L1*L2);
		if(argACos < -0.999 || argACos > 0.999)
		{
			error += 1;
			printf("ERROR: [in Kinematics::IKSolver] invalid cosine argument for the elbow pitch joint on arm %d\n", armID);
		}
		else
			*q4 = -acos(argACos);
	
		// (3) Shoulder pitch angle q1 (numeric method)
		if(error == 0)
			error = lookForQ1Solutions(x, y, z, q1_solutions, *q2);
		
		// (4) Shoulder yaw angle (q3)
		if(error == 0)
		{
			for(unsigned int k = 0; k < q1_solutions.size(); k++)
			{
				q1_aux = q1_solutions[k];
				double xw2 = x*cos(q1_aux) - z*sin(q1_aux);
				double yw2 = x*sin(q1_aux)*sin(*q2) + y*cos(*q2) + z*cos(q1_aux)*sin(*q2);
				q3_aux = atan2(yw2, xw2);
				// printf("Solution [%d]: <%.0lf, %.0lf, %.0lf, %.0lf> [deg]\n", k+1, 57.29*(q1_aux), 57.29*(*q2), 57.29*(q3_aux), 57.29*(*q4));
				if(q3_aux < 150 && q3_aux > -150 && checkSolution(x, y, z, q1_aux, *q2, q3_aux, *q4) == 0)
				{
					*q1 = q1_aux;
					*q3 = q3_aux;
				}
			}
		}
		
		// Check if solution is correct
		if(error == 0)	
			error = checkSolution(x, y, z, *q1, *q2, *q3, *q4);
			
		// Final time stamp
		gettimeofday(&tend, NULL);
		elapsedTime = (tend.tv_sec - tini.tv_sec) + 1e-6*(tend.tv_usec - tini .tv_usec);
		// printf("IK solver computational time: %.2lf [ms]\n", 1e3*elapsedTime);
	}
	
	
	return error;
}



int Kinematics::lookForQ1Solutions(double x, double y, double z, vector<double> & q1_solutions, double q2)
{
	double q1 = 0;
	double vk = 0;
	double vk_1 = 0;
	double w = 0;
	const double q1_step = 0.015708;
	const double epsilon_v = 1e-6;
	int error = 0;
	
	
	w = (L2*L2 - (L1*L1 + x*x + y*y + z*z) + 2*L1*y*sin(q2))/(2*L1*cos(q2));
	
	// Look for a positive value of q1 such that there is a change in the sign of v
	for(q1 = -0.95*1.5708; q1 < 0.95*1.5708; q1 += q1_step)
	{
		// Check the zero cross of v
		vk = x*sin(q1) + z*cos(q1) - w;
		vk_1 = x*sin(q1 - q1_step) + z*cos(q1 - q1_step) - w;
		if(fabs(vk) < epsilon_v || (vk_1 < 0 && vk > 0) || (vk_1 > 0 && vk < 0))
		{
			q1_solutions.push_back(q1);
			// printf("   Solution for Q1: %.0lf\n", 57.29*q1);
		}
	}
	
	if(q1_solutions.size() < 1)
	{
		error = 1;
		cout << "ERROR [in Kinematics::lookForQ1Solutions(...)]: no solution found for Q1" << endl;
	}
	
	// cout << "Numer of solutions found for Q1: " << q1_solutions.size() << endl;
	
	
	return error;
}




/*
 * Compare the Cartesian position obtained from the iverse kinematics with respect to the desired position
 */
int Kinematics::checkSolution(double x, double y, double z, double q1, double q2, double q3, double q4)
{
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double resultMatrix1[4][4];
	double resultMatrix2[4][4];
	double resultMatrix3[4][4];
	double p4_wrist[4];
	double p0_wrist[4];
	double x_ = 0;
	double y_ = 0;
	double z_ = 0;
	double ex = 0;
	double ey = 0;
	double ez = 0;
	double e = 0;	
	int error = 0;
	
	
	// Initialize matrices
	for(unsigned int row = 0; row < 4; row++)
	{
		for(unsigned int col = 0; col < 4; col++)
		{
			if(row != col)
			{
				T01[row][col] = 0;
				T12[row][col] = 0;
				T23[row][col] = 0;
				T34[row][col] = 0;
			}
			else
			{
				T01[row][col] = 1.0;
				T12[row][col] = 1.0;
				T23[row][col] = 1.0;
				T34[row][col] = 1.0;
			}
		}
	}
	
	// Set the value of the shoulder pitch transformation matrix
	T01[0][0] = cos(q1);
	T01[0][2] = sin(q1);
	T01[2][0] = -sin(q1);
	T01[2][2] = cos(q1);
	// Set the value of the shoulder roll transformation matrix
	T12[1][1] = cos(q2);
	T12[1][2] = -sin(q2);
	T12[2][1] = sin(q2);
	T12[2][2] = cos(q2);
	// Set the value of the shoulder yaw transformation matrix
	T23[0][0] = cos(q3);
	T23[0][1] = -sin(q3);
	T23[1][0] = sin(q3);
	T23[1][1] = cos(q3);
	// Set the value of the elbow pitch transformation matrix
	T34[0][0] = cos(q4);
	T34[0][2] = sin(q4);
	T34[2][0] = -sin(q4);
	T34[2][2] = cos(q4);
	T34[2][3] = -L1;
	// Set the value of the wrist point referred to the elbow pitch frame
	p4_wrist[0] = 0;
	p4_wrist[1] = 0;
	p4_wrist[2] = -L2;
	p4_wrist[3] = 1;
	
	// Compute the value of the wrist point referred to the shoulder pitch frame
	multiplyTransformationMatrices(T01, T12, resultMatrix1);
	multiplyTransformationMatrices(resultMatrix1, T23, resultMatrix2);
	multiplyTransformationMatrices(resultMatrix2, T34, resultMatrix3);
	multiplyTransformationMatrixVector(resultMatrix3, p4_wrist, p0_wrist);

	x_ = p0_wrist[0];
	y_ = p0_wrist[1];
	z_ = p0_wrist[2];
	
	// printf("<%.0lf, %.0lf, %.0lf, %.0lf> [deg] --> {%.0f, %.0f, %.0f} [cm]\n", 57.2958*q1, 57.2958*q2, 57.2958*q3, 57.2958*q4, 100*x_, 100*y_, 100*z_);
	
	ex = x - x_;
	ey = y - y_;
	ez = z - z_;
	e = sqrt(ex*ex + ey*ey + ez*ez);
	if(e > IK_SOLUTION_ERROR_LIMIT)
	{
		error = 1;
		printf("ERROR: [in Kinematics::checkSolution] FK/IK error too high (%.1lf [cm]) for arm %d\n", 100*e, armID);
	}

	/*
	printf("Joints position = {%.1lf, %.1lf, %.1lf, %.1lf} [deg]\n", 57.2958*q1, 57.2958*q2, 57.2958*q3, 57.2958*q4);
	printf("Input XYZ = {%.3lf, %.3lf, %.3lf}\n", x, y, z);
	printf("Output XYZ = {%.3lf, %.3lf, %.3lf}\n", x_, y_, z_);
	printf("Error = {%.3lf, %.3lf, %.3lf}\n", ex, ey, ez);
	printf("---\n");
	*/
	
	
	return error;
}




/*
 * Compare the Cartesian position obtained from the iverse kinematics with respect to the desired position
 */
int Kinematics::checkSolution_LinksLength(double x, double y, double z, double q1, double q2, double q3, double q4, double l1, double l2)
{
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double resultMatrix1[4][4];
	double resultMatrix2[4][4];
	double resultMatrix3[4][4];
	double p4_wrist[4];
	double p0_wrist[4];
	double x_ = 0;
	double y_ = 0;
	double z_ = 0;
	double ex = 0;
	double ey = 0;
	double ez = 0;
	double e = 0;	
	int error = 0;
	
	
	// Initialize matrices
	for(unsigned int row = 0; row < 4; row++)
	{
		for(unsigned int col = 0; col < 4; col++)
		{
			if(row != col)
			{
				T01[row][col] = 0;
				T12[row][col] = 0;
				T23[row][col] = 0;
				T34[row][col] = 0;
			}
			else
			{
				T01[row][col] = 1.0;
				T12[row][col] = 1.0;
				T23[row][col] = 1.0;
				T34[row][col] = 1.0;
			}
		}
	}
	
	// Set the value of the shoulder pitch transformation matrix
	T01[0][0] = cos(q1);
	T01[0][2] = sin(q1);
	T01[2][0] = -sin(q1);
	T01[2][2] = cos(q1);
	// Set the value of the shoulder roll transformation matrix
	T12[1][1] = cos(q2);
	T12[1][2] = -sin(q2);
	T12[2][1] = sin(q2);
	T12[2][2] = cos(q2);
	// Set the value of the shoulder yaw transformation matrix
	T23[0][0] = cos(q3);
	T23[0][1] = -sin(q3);
	T23[1][0] = sin(q3);
	T23[1][1] = cos(q3);
	// Set the value of the elbow pitch transformation matrix
	T34[0][0] = cos(q4);
	T34[0][2] = sin(q4);
	T34[2][0] = -sin(q4);
	T34[2][2] = cos(q4);
	T34[2][3] = -l1;
	// Set the value of the wrist point referred to the elbow pitch frame
	p4_wrist[0] = 0;
	p4_wrist[1] = 0;
	p4_wrist[2] = -l2;
	p4_wrist[3] = 1;
	
	// Compute the value of the wrist point referred to the shoulder pitch frame
	multiplyTransformationMatrices(T01, T12, resultMatrix1);
	multiplyTransformationMatrices(resultMatrix1, T23, resultMatrix2);
	multiplyTransformationMatrices(resultMatrix2, T34, resultMatrix3);
	multiplyTransformationMatrixVector(resultMatrix3, p4_wrist, p0_wrist);

	x_ = p0_wrist[0];
	y_ = p0_wrist[1];
	z_ = p0_wrist[2];
	
	// printf("<%.0lf, %.0lf, %.0lf, %.0lf> [deg] --> {%.0f, %.0f, %.0f} [cm]\n", 57.2958*q1, 57.2958*q2, 57.2958*q3, 57.2958*q4, 100*x_, 100*y_, 100*z_);
	
	ex = x - x_;
	ey = y - y_;
	ez = z - z_;
	e = sqrt(ex*ex + ey*ey + ez*ez);
	if(e > IK_SOLUTION_ERROR_LIMIT)
	{
		error = 1;
		printf("ERROR: [in Kinematics::checkSolution] FK/IK error too high (%.1lf [cm]) for arm %d\n", 100*e, armID);
	}

	/*
	printf("Joints position = {%.1lf, %.1lf, %.1lf, %.1lf} [deg]\n", 57.2958*q1, 57.2958*q2, 57.2958*q3, 57.2958*q4);
	printf("Input XYZ = {%.3lf, %.3lf, %.3lf}\n", x, y, z);
	printf("Output XYZ = {%.3lf, %.3lf, %.3lf}\n", x_, y_, z_);
	printf("Error = {%.3lf, %.3lf, %.3lf}\n", ex, ey, ez);
	printf("---\n");
	*/
	
	
	return error;
}


/*
 * Multiply 4x4 transformation matrices
 */
void Kinematics::multiplyTransformationMatrices(double M1[4][4], double M2[4][4], double resultMatrix[4][4])
{
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			resultMatrix[i][j] = 0;
			for(int k = 0; k < 4; k++)
				resultMatrix[i][j] += M1[i][k]*M2[k][j];
		}
	}
}
	
	
/*
 * Multiply a transformation matrix by a 4x1 vector
 **/
void Kinematics::multiplyTransformationMatrixVector(double M[4][4], double * V, double * resultVector)
{
	for(int i = 0; i < 4; i++)
	{
		resultVector[i] = 0;
		for(int j = 0; j < 4; j++)
			resultVector[i] += M[i][j]*V[j];
	}
}


int Kinematics::signDouble(double value)
{
	if(value > 0)
		return 1;
	else if(value < 0)
		return -1;
	else
		return 0;
}




