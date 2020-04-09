#include <iostream>
#include <cmath>
#include <Eigen/Dense>

Eigen::VectorXd	T2PR(const Eigen::MatrixXd& mT);
Eigen::MatrixXd PR_NJacobian(const Eigen::VectorXd& q);
Eigen::MatrixXd fk_fsc(const Eigen::VectorXd& q);
Eigen::MatrixXd DH(double twist_angle, double link_length, double joint_angle, double offset);
Eigen::MatrixXd TransX(double x);
Eigen::MatrixXd TransY(double y);
Eigen::MatrixXd TransZ(double z);
Eigen::MatrixXd RotX(double q);
Eigen::MatrixXd RotY(double q);
Eigen::MatrixXd RotZ(double q);

const int T = 1000;
const int N = 6;
const int M = 12;

int	main(int argc, char *argv[])
{
	int 	t = 0;
	const double c = 0.01;
	Eigen::MatrixXd Q(N,T + 1);

	//	Initial angle
	t = 0;
	Q.col(t) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	// Target T
	Eigen::MatrixXd mTargetT(4, 4);
	mTargetT <<
		1.0, 0.0, 0.0, 0.2,
		0.0, 1.0, 0.0, 0.2,
		0.0, 0.0, 1.0, 0.2,
		0.0, 0.0, 0.0, 1.0;
	Eigen::VectorXd vTargetPR(M);
	vTargetPR = T2PR(mTargetT);
	std::cout << "Target Poisition and Rotation" << std::endl
		<< vTargetPR << std::endl << std::endl;

	// Current T
	Eigen::MatrixXd mCurrentT(4, 4);
	Eigen::VectorXd vCurrentPR(M);
	mCurrentT = fk_fsc(Q.col(t));
	vCurrentPR = T2PR(mCurrentT);

	for(t = 0; t < T; t++)	{
		//	Make Jacobian

		Eigen::MatrixXd mJ(M, N);
		mJ = PR_NJacobian(Q.col(t));

		// Make psuedo inverse of Jacobian
		Eigen::MatrixXd	mJpinv(N,M);
		mJpinv = mJ.completeOrthogonalDecomposition().pseudoInverse();

		//	Update Joint angle and forward kinematics
		Q.col(t + 1) =  Q.col(t) + c * mJpinv * (vTargetPR - vCurrentPR);
		mCurrentT = fk_fsc(Q.col(t + 1));
		vCurrentPR = T2PR(mCurrentT);
	}

	std::cout << "Current Poisition and Rotation" << std::endl
		<< vCurrentPR << std::endl << std::endl;

	return(0);
}

Eigen::VectorXd	T2PR(const Eigen::MatrixXd& mT)
{
	Eigen::VectorXd vPR(M);

	//	[x; y; z]
	vPR.segment(0, 3) = mT.block<3,1>(0,3);
	//	[a^X_x; a^X_y; a^X_z]
	vPR.segment(3, 3) = mT.block<3,1>(0,0);
	//	[a^Y_x; a^Y_y; a^Y_z]
	vPR.segment(6, 3) = mT.block<3,1>(0,1);
	//	[a^Z_x; a^Z_y; a^Z_z]
	vPR.segment(9, 3) = mT.block<3,1>(0,2);

	return(vPR);
}

Eigen::MatrixXd PR_NJacobian(const Eigen::VectorXd& q)
{
	const double Dq = 1E-9;
	Eigen::MatrixXd mJ(M,N);
	Eigen::MatrixXd mT_q_Dq(4,4), mT_q(4,4), mDT(4,4);
	Eigen::VectorXd qD(N);

	for(int i = 0; i < N; i++)	{
		qD = q;
		qD(i) += Dq;
		mT_q  =  fk_fsc(q);
		mT_q_Dq =  fk_fsc(qD);
		mDT = (mT_q_Dq - mT_q) / Dq;
		mJ.block<3,1>(0,i) = mDT.block<3,1>(0,3);
		mJ.block<3,1>(3,i) = mDT.block<3,1>(0,0);
		mJ.block<3,1>(6,i) = mDT.block<3,1>(0,1);
		mJ.block<3,1>(9,i) = mDT.block<3,1>(0,2);
		mJ.col(i) = T2PR(mDT);
	}
	return(mJ);
}


Eigen::MatrixXd fk_fsc(const Eigen::VectorXd& q)
{
	const double D1 = 0.1;
	const double L1 = 0.1;
	const double L2 = 0.1;
	const double L3 = 0.1;
	const double L4 = 0.1;
	const double L5 = 0.1;

	Eigen::MatrixXd T01 = RotX(0.0)          * TransX(0.0) * RotZ(q(0)) * TransZ(D1);
	Eigen::MatrixXd T12 = RotX(EIGEN_PI/2.0) * TransX(L1)  * RotZ(q(1)) * TransZ(0);
	Eigen::MatrixXd T23 = RotX(EIGEN_PI/2.0) * TransX(L2)  * RotZ(q(2)) * TransZ(0);
	Eigen::MatrixXd T34 = RotX(EIGEN_PI/2.0) * TransX(L3)  * RotZ(q(3)) * TransZ(0);
	Eigen::MatrixXd T45 = RotX(EIGEN_PI/2.0) * TransX(L4)  * RotZ(q(4)) * TransZ(0);
	Eigen::MatrixXd T56 = RotX(EIGEN_PI/2.0) * TransX(L5)  * RotZ(q(5)) * TransZ(0);
	Eigen::MatrixXd T02 = T01 * T12;
	Eigen::MatrixXd T03 = T02 * T23;
	Eigen::MatrixXd T04 = T03 * T34;
	Eigen::MatrixXd T05 = T04 * T45;
	Eigen::MatrixXd T06 = T05 * T56;

	return(T06);
}

Eigen::MatrixXd DH(double twist_angle, double link_length, double joint_angle, double offset)
{
	Eigen::MatrixXd m(4, 4);

	m = RotX(twist_angle) * TransX(link_length) * RotZ(joint_angle) * TransZ(offset);
	return(m);
}


Eigen::MatrixXd TransX(double x)
{
	Eigen::MatrixXd m(4, 4);

	m << 1.0, 0.0, 0.0, x,
		 0.0, 1.0, 0.0, 0.0,
		 0.0, 0.0, 1.0, 0.0,
		 0.0, 0.0, 0.0, 1.0;

	return(m);
}

Eigen::MatrixXd TransY(double y)
{
	Eigen::MatrixXd m(4, 4);

	m << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,   y,
		 0.0, 0.0, 1.0, 0.0,
		 0.0, 0.0, 0.0, 1.0;

	return(m);
}

Eigen::MatrixXd TransZ(double z)
{
	Eigen::MatrixXd m(4, 4);

	m << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 0.0, 0.0, 1.0,   z,
		 0.0, 0.0, 0.0, 1.0;

	return(m);
}


Eigen::MatrixXd RotX(double q)
{
	Eigen::MatrixXd m(4, 4);

	m << 1.0,    0.0,     0.0, 0.0,
		 0.0, cos(q), -sin(q), 0.0,
		 0.0, sin(q),  cos(q), 0.0,
		 0.0,    0.0,     0.0, 1.0;

	return(m);
}

Eigen::MatrixXd RotY(double q)
{
	Eigen::MatrixXd m(4, 4);

	m <<  cos(q), 0.0, sin(q), 0.0,
	         0.0, 1.0,    0.0, 0.0,
		 -sin(q), 0.0, cos(q), 0.0,
	         0.0, 0.0,    0.0, 1.0;

	return(m);
}

Eigen::MatrixXd RotZ(double q)
{
	Eigen::MatrixXd m(4, 4);

	m << cos(q), -sin(q), 0.0, 0.0,
		 sin(q),  cos(q), 0.0, 0.0,
		    0.0,     0.0, 1.0, 0.0,
		    0.0,     0.0, 0.0, 1.0;

	return(m);
}
