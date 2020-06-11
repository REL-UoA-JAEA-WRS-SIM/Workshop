/*
	XY2LinkController3.cpp
		Solve inverse kinematics in control() by numerical Jacobain
		Author: Keitaro Naruse
		Date: 2020-06-10
*/

#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <cnoid/SimpleController>

using namespace cnoid;

class XY2LinkController3 : public SimpleController
{
private:
	//  Pointers to link structure for 2link robot arm
	//  2つのリンクのためのリンク構造体へのポインタ
    Link* joint[2];
    
    //  Reference angle of each of the six joints
    //  2つの関節のそれぞれの目標角度
    double q_ref[2];

    //  Joint angles of each of the six joints at the previous time step
    //  2つの関節のそれぞれの必つ前の時刻での関節角度
    double q_prev[2];
    //  Delta t
    //  時間の刻み幅
    double dt;

    //  Goal hand position
    //  手先の目標位置
    double p_ref[2];

    /*
        foward_kinematics()
            Input:	q, Joint angle vector = {q1, q2}
            Output:	p, hand position vector = {x, y}
    */
    Eigen::Vector2d forward_kinematics(const Eigen::Vector2d& q)
    {
        //  Link parameters
        //  リンクパラメータ
        const double L1 = 0.1;
        const double L2 = 0.1;
        //	Hand position vector
        //  手先の位置ベクトル
        Eigen::Vector2d	p;

        //  x component
        //  x座標の計算
        p(0) = L1 * cos(q(0)) + L2 * cos(q(0) + q(1));
        //  y component        
        //  y座標の計算
        p(1) = L1 * sin(q(0)) + L2 * sin(q(0) + q(1));

        //  Returns hand position vector
        //  手先の位置ベクトルを戻す
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
        const double dq = 1E-9;
        Eigen::Matrix2d	j;
        Eigen::Vector2d dq1(dq, 0), dq2(0, dq);
        Eigen::Vector2d p, pdp1, pdp2;

        //	forward kinematics of the original angle vector q
        //	元のqによる手先の位置ベクトル
        p = forward_kinematics(q);
        //	forward kinematics of the angle vector q+dq1 (q1 is increased a small amount) 
        //	q+dq1 (q1がごく少量増加)による手先の位置ベクトル
        pdp1 = forward_kinematics(q + dq1);
        //	forward kinematics of the angle vector q+dq2 (q2 is increased a small amount) 
        //	q+dq2 (q2がごく少量増加)による手先の位置ベクトル
        pdp2 = forward_kinematics(q + dq2);

        //	dx/dt = ((x+dx)-x)/((q+dq1) - q1) 
        //	dy/dt = ((y+dy)-y)/((q+dq1) - q1) 
        j.col(0) = (pdp1 - p) / dq;
        //	dx/dt = ((x+dx)-x)/((q+dq2) - q2) 
        //	dy/dt = ((y+dy)-y)/((q+dq2) - q2) 
        j.col(1) = (pdp2 - p) / dq;

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
            if (1E-9 > fabs(sigma(i))) {
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
        m_psuedo_inverse = svd.matrixV() * sigma_inverse.asDiagonal() * svd.matrixU().transpose();

        return(m_psuedo_inverse);
    }


public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        //  Get a joint from a body 
        //  関節をボディモデルから取得
        joint[0] = io -> body() -> link("LINK_1");
        joint[1] = io -> body() -> link("LINK_2");
        
        for(int i = 0; i < 2; i++)	{
            //  Set a joint as torque mode
            //  関節をトルクモードの設定する
            joint[i] -> setActuationMode(Link::JOINT_TORQUE);
            //  Set a joint controllable
            //  関節を制御可能に設定する
            io -> enableIO(joint[i]);
        }
        //  Get a time period of simulation
        //  シミュレーションの時間の刻み幅を取得する
        dt = io -> timeStep();
        
        //  Set a reference hand position vector
        //  手先の目標位置を設定する
        //  as an ordinary array in c++
        p_ref[0] = 0.0;
        p_ref[1] = 0.1;
        return true;
    }
    
    virtual bool control() override
    {
        //  Inverse kinematics solution part: We find q_ref
        //  逆運動学計算部分: q_refを求める

        //	Iteration number
        //	繰り返し回数
        const int T = 100;
        //	Time idenx
        //	時間の添え字
        int	t;
        //	Coefficient in inverse kinematics solution
        //	逆運動学計算のための係数
        double	a = 0.1;

        //  Reference hand position vector 
        //  手先の目標位置
        Eigen::Vector2d pg;
        //  Current joint angle vector: {q1, q2} [rad]
        //  現在の関節角度ベクトル
        Eigen::Vector2d	q;
        //  Current Hand position vector: {x, y} [m]
        //  現在の手先の位置ベクトル
        Eigen::Vector2d	p;
        //  Jacobain
        //  ヤコビアン
        Eigen::Matrix2d j, pinv;

        //  as a vector in Eigen
        pg(0) = p_ref[0];
        pg(1) = p_ref[1];

        //	Set a current joint angle to a vector for inverse kinematic ssolution
        //	現在のボディモデルの関節角度を逆運動学計算用の関節角度ベクトルに与える
        q(0) = joint[0]->q(); // input
        q(1) = joint[1]->q(); // input


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
            q += a * pseudo_inverse(j) * (pg - p);
        }

        //  Set a calculated joint angle to reference joint angle
        //  計算された関節角度を参照関節角度とする
        q_ref[0] = fmod(q(0), 2 * M_PI);
        q_ref[1] = fmod(q(1), 2 * M_PI);

        //  Control part part: We find q_ref
        //  Control a joint to a reference angle
    
        //  PD gains
        //  PD制御のためのゲイン
        static const double P[2] = { 10.0, 10.0 };
        static const double D[2] = { 5.0,  5.0 };

        //  Joint angle
        //  関節角度
        double q_curr[2];
        //  Joint angular velocity
        //  関節角速度
        double dq[2];
        //  Reference joint angular velocity
        //  参照関節角速度
        double dq_ref[2] = {0.0, 0.0};

        for (int i = 0; i < 2; i++) {
            //  Get a joint angle from a body model
            //  ボディモデルから関節角度を取得する
            q_curr[i] = joint[i] -> q(); // input
            //  Calculate a joint angular velocity
            //  １時刻前の関節角度を使って，関節角速度を計算する
            dq[i] = (q_curr[i] - q_prev[i]) / dt;
            //  Determine a joint torque 
            //  PD制御で関節トルクを決定する
            joint[i]->u() = P[i] * (q_ref[i] - q_curr[i]) + D[i] * (dq_ref[i] - dq[i]); // output

            //  Store a current joint angle as a previous one
            //  現在の関節角度を一つ前のものとして保存する
            q_prev[i] = q_curr[i];
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(XY2LinkController3)
