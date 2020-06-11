/*
    XYZ3LinkController1.cpp
        Solve inverse kinematics in control() by numerical Jacobain
        Author: Keitaro Naruse
        Date: 2020-06-10
*/

#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <cnoid/SimpleController>

using namespace cnoid;

class XYZ3LinkController1 : public SimpleController
{
private:
    //  Pointers to link structure for 3link robot arm
    //  3つのリンクのためのリンク構造体へのポインタ
    Link* joint[3];

    //  Reference angle of each of the three joints
    //  3つの関節のそれぞれの目標角度
    double q_ref[3];

    //  Joint angles of each of the three joints at the previous time step
    //  3つの関節のそれぞれの必つ前の時刻での関節角度
    double q_prev[3];
    //  Delta t
    //  時間の刻み幅
    double dt;

    //  Goal hand position
    //  手先の目標位置
    double p_ref[3];

    /*
       foward_kinematics()
           Input:	q, Joint angle vector = {q1, q2, q3}
           Output:	p, hand position vector = {x, y, z}
   */
    Eigen::Vector3d forward_kinematics(const Eigen::Vector3d& q)
    {
        //  Link parameters
        const double L1 = 0.1;
        const double L2 = 0.1;
        const double L3 = 0.1;

        //	Homogeneous transformation matrix
        Eigen::Matrix4d	t01, t12, t23, t34, t04;
        t01 = DH(0.0, 0.0, q(0), 0.0);
        t12 = DH(M_PI_2, L1, q(1), 0.0);
        t23 = DH(0.0, L2, q(2), 0.0);
        t34 = DH(0.0, L3, 0.0, 0.0);
        t04 = t01 * t12 * t23 * t34;

        //	Homogeneous vector
        const Eigen::Vector4d	o(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4d	p = t04 * o;

        return(p.head(3));
    }

    /*
        DH()
            Input:	DH patameters
            Output:	t, 4*4 Homogeneous transformation matrix

    */
    Eigen::Matrix4d	DH(double twist, double length, double angle, double offset)
    {
        Eigen::Matrix4d	t;

        t(0, 0) = cos(angle);
        t(0, 1) = -sin(angle);
        t(0, 2) = 0.0;
        t(0, 3) = length;

        t(1, 0) = sin(angle) * cos(twist);
        t(1, 1) = cos(angle) * cos(twist);
        t(1, 2) = -sin(twist);
        t(1, 3) = -offset * sin(twist);

        t(2, 0) = sin(angle) * sin(twist);
        t(2, 1) = cos(angle) * sin(twist);
        t(2, 2) = cos(twist);
        t(2, 3) = offset * cos(twist);

        t(3, 0) = 0.0;
        t(3, 1) = 0.0;
        t(3, 2) = 0.0;
        t(3, 3) = 1.0;

        return(t);
    }

    /*
        jacobian()
            Find Jacobian numerically
            (Give Jacobian by an increment of a small displacement of forward kinematics)

            Input:	q, Joint angle vevtor = {q1, q2, q3}
            Output:	j, Jacobain

    */
    Eigen::Matrix3d jacobian(const Eigen::Vector3d& q)
    {
        const double dq = 1E-9;
        Eigen::Matrix3d	j;
        Eigen::Vector3d dq1(dq, 0.0, 0.0), dq2(0.0, dq, 0.0), dq3(0.0, 0.0, dq);
        Eigen::Vector3d p, pdp1, pdp2, pdp3;

        //	forward kinematics of the original angle vector q
        //	元のqによる手先の位置ベクトル
        p = forward_kinematics(q);
        //	forward kinematics of the angle vector q+dq1 (q1 is increased a small amount) 
        //	q+dq1 (q1がごく少量増加)による手先の位置ベクトル
        pdp1 = forward_kinematics(q + dq1);
        //	forward kinematics of the angle vector q+dq2 (q2 is increased a small amount) 
        //	q+dq2 (q2がごく少量増加)による手先の位置ベクトル
        pdp2 = forward_kinematics(q + dq2);
        //	forward kinematics of the angle vector q+dq3 (q3 is increased a small amount) 
        //	q+dq3 (q3がごく少量増加)による手先の位置ベクトル
        pdp3 = forward_kinematics(q + dq3);

        //	dx/dt = ((x+dx)-x)/((q+dq1) - q1) 
        //	dy/dt = ((y+dy)-y)/((q+dq1) - q1) 
        //	dz/dt = ((z+dz)-z)/((q+dq1) - q1) 
        j.col(0) = (pdp1 - p) / dq;
        //	dx/dt = ((x+dx)-x)/((q+dq2) - q2) 
        //	dy/dt = ((y+dy)-y)/((q+dq2) - q2) 
        //	dz/dt = ((z+dz)-z)/((q+dq2) - q2) 
        j.col(1) = (pdp2 - p) / dq;
        //	dx/dt = ((x+dx)-x)/((q+dq3) - q3) 
        //	dy/dt = ((y+dy)-y)/((q+dq3) - q3) 
        //	dz/dt = ((z+dz)-z)/((q+dq3) - q3) 
        j.col(2) = (pdp3 - p) / dq;

        return(j);
    }

    /*
        pseudo_inverse()
            Input:	m, 3*3 matirx
            Output:	pseudo inverse of a given m
    */
    Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d& m)
    {
        //	Psuedo inverse matrix
        //	疑似逆行列
        Eigen::Matrix3d	m_psuedo_inverse;
        //	Sinvgular Value Decomposition of a given m
        //	与えられた行列の特異値分解
        Eigen::JacobiSVD< Eigen::Matrix3d >	svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //	Diagonal vector of singular value matrix and inverse singular value matrix
        //	特異値行列と逆特異値行列のの対角成分
        Eigen::Vector3d	sigma, sigma_inverse;
        sigma = svd.singularValues();

        //	Make an inverse singular value matrix from singular value one
        //	特異値行列の逆数から逆特異値行列を作る
        for (int i = 0; i < 3; ++i) {
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

    /*
        inverse_kinemaitcs()
            Input:	pg[2], goal hand position {x, y}
                    q0[2], initial joint angles {q0_1, q0_2}
            Output:	qg[2], goal joint angles {qg_1, qg_2}
    */
    Eigen::Vector3d inverse_kinematics(const Eigen::Vector3d& pg, const Eigen::Vector3d& qi)
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
        Eigen::Vector3d	p;
        //	Current joint agngle vector
        //	現在の関節角度ベクトル
        Eigen::Vector3d	q;
        //	Joint angle update vector
        //	関節角度更新量ベクトル
        Eigen::Vector3d	qd;
        //	Jocobian
        //	ヤコビアン
        Eigen::Matrix3d	j;

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
            //  q += a * j.inverse() * (pg - p);

            //	Instead, I use psuedo inverse
            //	代わりに疑似逆行列を用いる
            q += a * pseudo_inverse(j) * (pg - p);
        }

        //	final joint angles
        //	最終的に得られた関節角度
        return(q);
    }

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        //  Get a joint from a body 
        //  関節をボディモデルから取得
        joint[0] = io->body()->link("LINK_1");
        joint[1] = io->body()->link("LINK_2");
        joint[2] = io->body()->link("LINK_3");

        for (int i = 0; i < 3; i++) {
            //  Set a joint as torque mode
            //  関節をトルクモードの設定する
            joint[i]->setActuationMode(Link::JOINT_TORQUE);
            //  Set a joint controllable
            //  関節を制御可能に設定する
            io->enableIO(joint[i]);
        }
        //  Get a time period of simulation
        //  シミュレーションの時間の刻み幅を取得する
        dt = io->timeStep();

        //  Set a reference hand position vector
        //  手先の目標位置を設定する
        //  as an ordinary array in c++
        p_ref[0] = 0.0;
        p_ref[1] = 0.1;
        p_ref[2] = 0.1;
        return true;
    }

    virtual bool control() override
    {
        //	Current hand position vector{x, y}
        //	現在の手先の位置ベクトル
        Eigen::Vector3d	p;
        //	現在の手先の目標位置ベクトル
        Eigen::Vector3d	pg;
        //	Current joint agngle vector
        //	現在の関節角度ベクトル
        Eigen::Vector3d	q;
        //	Joint angle update vector
        //	目標関節角度ベクトル
        Eigen::Vector3d	qf;
        //  Inverse kinematics solution part: We find q_ref
        //  逆運動学計算部分: q_refを求める

        //  as a vector in Eigen
        pg(0) = p_ref[0];
        pg(1) = p_ref[1];
        pg(2) = p_ref[2];

        //	Set a current joint angle to a vector for inverse kinematic ssolution
        //	現在のボディモデルの関節角度を逆運動学計算用の関節角度ベクトルに与える
        q(0) = joint[0]->q(); // input
        q(1) = joint[1]->q(); // input
        q(2) = joint[2]->q(); // input

        qf = inverse_kinematics(pg, q);

        //  Set a calculated joint angle to reference joint angle
        //  計算された関節角度を参照関節角度とする
        q_ref[0] = fmod(qf(0), 2 * M_PI);
        q_ref[1] = fmod(qf(1), 2 * M_PI);
        q_ref[2] = fmod(qf(2), 2 * M_PI);

        //  Control part part: We find q_ref
        //  Control a joint to a reference angle

        //  PD gains
        //  PD制御のためのゲイン
        static const double P[3] = { 10.0, 10.0, 10.0 };
        static const double D[3] = { 5.0,  5.0,   5.0 };

        //  Joint angle
        //  関節角度
        double q_curr[3];
        //  Joint angular velocity
        //  関節角速度
        double dq[3];
        //  Reference joint angular velocity
        //  参照関節角速度
        double dq_ref[3] = { 0.0, 0.0, 0.0 };

        for (int i = 0; i < 3; i++) {
            //  Get a joint angle from a body model
            //  ボディモデルから関節角度を取得する
            q_curr[i] = joint[i]->q(); // input
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

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(XYZ3LinkController1)
