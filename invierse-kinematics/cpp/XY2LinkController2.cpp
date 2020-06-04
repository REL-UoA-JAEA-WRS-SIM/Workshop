/*
	XY2LinkController2.cpp
		Control each of the joints to a given angle
        Author: Keitaro Naruse
		Date: 2020-06-02
*/

#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <cnoid/SimpleController>

using namespace cnoid;

class XY2LinkController2 : public SimpleController
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
    Eigen::Vector2d	pg;
    //  Current joint angle vector: {q1, q2} [rad]
    //  現在の関節角度ベクトル
    //Eigen::Vector2d	q;
    //  Previous joint angle vector: {q1, q2} [rad]
    //  直前の関節角度ベクトル
    //Eigen::Vector2d	q_prev;

    //  Reference joint angle vector by inverse kinematics solution
    //  逆運動学による参照（目標）関節角度ベクトル
    //Eigen::Vector2d	q_ref;
    //  Current Hand position vector: {x, y} [m]
    //  現在の手先の位置ベクトル
    //Eigen::Vector2d	p;
    //  Jacobain
    //Eigen::Matrix2d j;

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
        //  リンクパラメータ
        const double L1 = 0.1;
        const double L2 = 0.1;
        Eigen::Matrix2d	j;
        
        //  Jacobian is given as follows
        //  ヤコビアンは次の式で与えらえる
        j(0, 0) = -L1 * sin(q(0)) - L2 * sin(q(0) + q(1));
        j(0, 1) = -L2 * sin(q(0) + q(1));
        j(1, 0) = L1 * cos(q(0)) + L2 * cos(q(0) + q(1));
        j(1, 1) = L2 * cos(q(0) + q(1));

        return(j);
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
        pg(0) = 0.0;
        pg(1) = 0.1;

        //  Inverse kinematics solution
        //	Iterated calculation
        //const int T = 100;
        //  coefficient of inverse kinematics
        //  逆運動学の数値計算のための係数
        //const double a = 0.1;
        //for (int t = 0; t < T; ++t) {
        //    //	forward kinematics
        //    p = forward_kinematics(q);
        //    //	Find Jacobian
        //    j = jacobian(q);
        //    //	Update a joint angle vector
        //    q = q + a * j.inverse() * (pg - p);
        //}
        //q_ref = q;

        q_ref[0] = 0.523556;
        q_ref[1] = 2.09442;

        return true;
    }
    
    virtual bool control() override
    {
        // PD gains
        static const double P[2] = { 10.0, 10.0 };
        static const double D[2] = { 5.0,  5.0 };

        double q[2];
        double dq[2];
        double dq_ref[2];

        for (int i = 0; i < 2; i++) {
            q[i] = joint[i]->q(); // input
            dq[i] = (q[i] - q_prev[i]) / dt;
            dq_ref[i] = 0.0;
            joint[i]->u() = P[i] * (q_ref[i] - q[i]) + D[i] * (dq_ref[i] - dq[i]); // output
            q_prev[i] = q[i];
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(XY2LinkController2)
