/*
	ik_2link_symJ_eigen_cpp
		Sample program for inverse kinematics of 2-link robot arm with Eigen
		Author: Keitaro Naruse
		Date: 2020-06-01
*/

#define _USE_MATH_DEFINES // for C++

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/SVD>

//  Function prototypes
Eigen::Vector2d inverse_kinematics(const Eigen::Vector2d& pg, const Eigen::Vector2d& qi);
Eigen::Vector2d forward_kinematics(const Eigen::Vector2d& q);
Eigen::Matrix2d jacobian(const Eigen::Vector2d& q);

/*
	main()
*/

int main()
{
	//  Goal hand position
	Eigen::Vector2d	pg(0.0, 0.1);
	//  Initial joint angle: {q1, q2} [rad]
	Eigen::Vector2d	qi(0.0, 0.0);
	//  Final joint angle: {q1, q2} [rad]
	Eigen::Vector2d	qf;
	//  Final hand position: {x, y} [m]
	Eigen::Vector2d	pf;

	Eigen::Matrix2d	j;

	//	Solve inverse kinematics iteratively
	qf = inverse_kinematics(pg, qi);
	//	Forward kinematics for found joint angle vector
	pf = forward_kinematics(qf);

	//  Show initial joint angles
	std::cout << "Initial joint angles" << std::endl << qi << std::endl;
	//  Show a goal hand position;
	std::cout << "Goal hand position" << std::endl << pg << std::endl;
	//  Show final joint angles
	std::cout << "Final joint angles" << std::endl << qf << std::endl;
	//  Show a goal hand position;
	std::cout << "Final hand position" << std::endl << pf << std::endl;

	return(0);
}

/*
	foward_kinematics()
		Input:	q, Joint angle vector = {q1, q2}
		Output:	p, hand position vector = {x, y}
*/
Eigen::Vector2d forward_kinematics(const Eigen::Vector2d& q)
{
	//  Link parameters
	const double L1 = 0.1;
	const double L2 = 0.1;
	//	Hand position vector
	Eigen::Vector2d	p;

	//  x component
	p(0) = L1 * cos(q(0)) + L2 * cos(q(0) + q(1));
	//  y component
	p(1) = L1 * sin(q(0)) + L2 * sin(q(0) + q(1));

	return(p);
}


/*
	jacobian()
		Input:	q, Joint angle vevtor = {q1, q2}
		Output:	j, Jacobain
*/
Eigen::Matrix2d jacobian(const Eigen::Vector2d& q)
{
	//  Link parameters
	const double L1 = 0.1;
	const double L2 = 0.1;
	Eigen::Matrix2d	j;

	j(0, 0) = -L1 * sin(q(0)) - L2 * sin(q(0) + q(1));
	j(0, 1) = -L2 * sin(q(0) + q(1));
	j(1, 0) = L1 * cos(q(0)) + L2 * cos(q(0) + q(1));
	j(1, 1) = L2 * cos(q(0) + q(1));

	return(j);
}

/*
	pseudo_inverse()
		Input:	m, 2*2 matirx
		Output:	pseudo inverse of a given m
*/
Eigen::Matrix2d pseudo_inverse(const Eigen::Matrix2d& m)
{
	//	Psuedo inverse matrix
	//	疑似逆行列
	Eigen::Matrix2d	m_psuedo_inverse;
	//	Sinvgular Value Decomposition of a given m
	//	与えられた行列の特異値分解
	Eigen::JacobiSVD< Eigen::Matrix2d >	svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//	Diagonal vector of singular value matrix and inverse singular value matrix
	//	特異値行列と逆特異値行列のの対角成分
	Eigen::Vector2d	sigma, sigma_inverse;
	sigma = svd.singularValues();

	//	Make an inverse singular value matrix from singular value one
	//	特異値行列の逆数から逆特異値行列を作る
	for (int i = 0; i < 2; ++i) {
		//	If a diagonal componets is very small, set sigma_inverse as 0
		//	対角成分の値が小さかったら，０を与える
		if (1E-9 > fabs(sigma(i)) ) {
			sigma_inverse(i) = 0.0;
		}
		//	If a diagonal componets is large enoughr, set sigma_inverse as an inverse of it
		//	対角成分の値が十分大きかったら，特異値の逆数を与える
		else
		{
			sigma_inverse(i) = 1.0 / sigma(i);
		}
	}
	//	Calulate psuedo inverse matrix from singular value decomposition
	m_psuedo_inverse = svd.matrixV() *  sigma_inverse.asDiagonal() * svd.matrixU().transpose();

	return(m_psuedo_inverse);
}

/*
	inverse_kinemaitcs()
		Input:	pg[2], goal hand position {x, y}
				q0[2], initial joint angles {q0_1, q0_2}
		Output:	qg[2], goal joint angles {qg_1, qg_2}
*/
Eigen::Vector2d inverse_kinematics(const Eigen::Vector2d& pg, const Eigen::Vector2d& qi)
{
	//	Iteration number
	//	繰り返し回数
	const int T = 100;
	//	Time idenx
	//	時間の添え字
	int	t;
	//	Coefficient in inverse kinematics solution
	//	逆運動学計算のための係数
	double	a = 0.1;

	//	Current hand position vector{x, y}
	//	現在の手先の位置ベクトル
	Eigen::Vector2d	p;
	//	Current joint agngle vector
	//	現在の関節角度ベクトル
	Eigen::Vector2d	q;
	//	Joint angle update vector
	//	関節角度更新量ベクトル
	Eigen::Vector2d	qd;
	//	Jocobian
	//	ヤコビアン
	Eigen::Matrix2d	j;

	//	Set an initial joint angle vetor to a current one
	//	初期関節角度を現在の関節角度ベクトルに与える
	q = qi;

	//	Iterated calculation for innverse kinematics
	//	逆運動学のための繰り返し計算
	for (t = 0; t < T; ++t) {
		//	forward kinematics
		//	順運動学
		p = forward_kinematics(q);
		//	Find Jacobian
		//ヤコビアン
		j = jacobian(q);
		//	Update joint angles
		//	関節角度の更新

		//	Simple inverse matrix does not work in a singular pose
		//	単純な逆行列だと特異姿勢のときに逆行列が定まらない
		//	q += a * j.inverse() * (pg - p);

		//	Instead, I use psuedo inverse
		//	代わりに疑似逆行列を用いる
		q +=  a * pseudo_inverse(j) * (pg - p);
	}

	//	final joint angles
	//	最終的に得られた関節角度
	return(q);
}

