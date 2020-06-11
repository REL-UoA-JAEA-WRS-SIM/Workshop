// ik_2link_pure_cpp.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

/*
	ik_2link_pure_cpp
		Sample program for inverse kinematics of 2-link robot arm by pure C++
		Author: Keitaro Naruse
		Date: 2020-06-01
*/

#define _USE_MATH_DEFINES // for C++

#include <iostream>
#include <cmath>

//  Function prototypes
void inverse_kinematics(double pg[2], double q0[2], double qg[2]);
void forward_kinematics(double q[2], double p[2]);
void jacobian(double q[2], double j[2][2]);
void inverse(double j[2][2], double jinv[2][2]);

int main()
{
	//  Goal hand position
	double pg[2] = { 0.0, 0.1 };

	//  Initial joint angle: {q1, q2} [rad]
	double qi[2] = {0.1, 0.1};
	//  Final joint angle: {q1, q2} [rad]
	double qf[2];
	//  Final hand position: {x, y} [m]
	double pf[2];

	//  Show initial joint angles
	std::cout << "Initial joint angles" << std::endl;
	std::cout << "qi_1: " << qi[0] << std::endl;
	std::cout << "qi_2: " << qi[1] << std::endl;

	//  Show a goal hand position;
	std::cout << "Goal hand position" << std::endl;
	std::cout << "x: " << pg[0] << std::endl;
	std::cout << "y: " << pg[1] << std::endl;

	//	Solve inverse kinematics iteratuively 
	inverse_kinematics(pg, qi, qf);
	//	Calculate final hand position 
	forward_kinematics(qf, pf);

		//  Show final joint angles
	std::cout << "Final joint angles" << std::endl;
	std::cout << "qf_1: " << qf[0] << std::endl;
	std::cout << "qf_2: " << qf[1] << std::endl;

	//  Show final hand position
	std::cout << "Final hand position" << std::endl;
	std::cout << "xf: " << pf[0] << std::endl;
	std::cout << "yf: " << pf[1] << std::endl;

	return(0);
}

/*
	inverse_kinemaitcs()
		Input:	pg[2], goal hand position {x, y}
				q0[2], initial joint angles {q0_1, q0_2}
		Output:	qg[2], goal joint angles {qg_1, qg_2} 
*/
void inverse_kinematics(double pg[2], double q0[2], double qg[2])
{
	//	Iteration number
	const int T = 100;
	int		t;
	//	Update coefficient
	double	a = 0.1;

	//	Current hand position {x, y}
	double p[2];
	//	Difference between desired and current hand position
	double pd[2];
	//	Current joint agngles
	double q[2];
	//	Update amount of joint agngles
	double qd[2];

	//	Jocobian
	double j[2][2];
	//	Inverse of Jocobian
	double jinv[2][2];

	//	Set initial joint angles to current joint angles
	q[0] = q0[0];
	q[1] = q0[1];

	//	Iterated calculation
	for (t = 0; t < T; ++t) {
		//	forward kinematics
		forward_kinematics(q, p);
		//	Calculate a difference between goal and current hand position
		pd[0] = pg[0] - p[0];
		pd[1] = pg[1] - p[1];
		//	Find Jacobian
		jacobian(q, j);
		//	Find inverse of Jacobian
		inverse(j, jinv);
		//	Calculate an update amount of joint angles
		qd[0] = a * (jinv[0][0] * pd[0] + jinv[0][1] * pd[1]);
		qd[1] = a * (jinv[1][0] * pd[0] + jinv[1][1] * pd[1]);
		//	Update joint angles 
		q[0] = q[0] + qd[0];
		q[1] = q[1] + qd[1];
	}
	//	final joint angles
	qg[0] = q[0];
	qg[1] = q[1];
}

/*
	foward_kinematics()
		Input:	q[2], Joint angles, q1, q2
		Output:	p[2], hand position in x-y plane, x, y
*/
void forward_kinematics(double q[2], double p[2])
{
	//  Link parameters
	const double L1 = 0.1;
	const double L2 = 0.1;

	//  x component
	p[0] = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]);
	//  y component
	p[1] = L1 * sin(q[0]) + L2 * sin(q[0] + q[1]);
}

/*
	jacobian()
		Input:	q[2], Joint angles, q1, q2
		Output:	j[2][2], jacobain
*/
void jacobian(double q[2], double j[2][2])
{
	//  Link parameters
	const double L1 = 0.1;
	const double L2 = 0.1;

	j[0][0] = -L1 * sin(q[0]) - L2 * sin(q[0] + q[1]);
	j[0][1] = -L2 * sin(q[0] + q[1]);
	j[1][0] = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]);
	j[1][1] = L2 * cos(q[0] + q[1]);
}

/*
	inverse()
		Input:	j[2][2]
		Output:	jinv[2][2]
*/
void inverse(double j[2][2], double jinv[2][2])
{
	double	det = j[0][0] * j[1][1] - j[0][1] * j[1][0];

	jinv[0][0] = j[1][1] / det;
	jinv[0][1] = -j[0][1] / det;
	jinv[1][0] = -j[1][0] / det;
	jinv[1][1] = j[0][0] / det;
}


