/*
	XY2LinkController0.cpp
		Control each of the joints to a given angle
        Author: Keitaro Naruse
		Date: 2020-06-11
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class XY2LinkController0 : public SimpleController
{
	//  Pointers to link structure for six joitns
	//  2つのリンクのためのリンク構造体へのポインタ
    Link* joint[2];
    
    //  Reference angle of each of the two joints
    //  2つの関節のそれぞれの目標角度
    double q_ref[2];
    
    //  Joint angles of each of the two joints at the previous time step
    //  2つの関節のそれぞれの必つ前の時刻での関節角度
    double q_prev[2];
    
    //  Delta t
    //  時間の刻み幅
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        joint[0] = io->body()->link("LINK_1");
        joint[1] = io->body()->link("LINK_2");
        
        for(int i = 0; i < 2; i++)	{
            joint[i]->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint[i]);
            //  Given angle
            q_ref[i] = q_prev[i] = M_PI / 6.0;
        }
        dt = io->timeStep();
        
        return true;
    }
    
    virtual bool control() override
    {
        // PD gains
        static const double P[2] = { 10.0, 10.0 };
        static const double D[2] = {  5.0,  5.0 };
       
        double q[2];
        double dq[2];
        double dq_ref[2];
        
        for(int i = 0; i < 2; i++)	{
            q[i]  = joint[i]->q(); // input
            dq[i] = (q[i] - q_prev[i]) / dt;
            dq_ref[i] = 0.0;
            joint[i]->u() = P[i] * (q_ref[i] - q[i]) + D[i] * (dq_ref[i] - dq[i]); // output
            q_prev[i] = q[i];
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(XY2LinkController0)
