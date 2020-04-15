/*
	FSCController1.cpp
		Remain current joint angles
		Author: Keitaro Naruse
		Date: 2020-04-15
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class FSCController2 : public SimpleController
{
    Link* joint[6];
    double q_ref[6];
    double q_prev[6];
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        joint[0] = io->body()->link("LINK_1");
        joint[1] = io->body()->link("LINK_2");
        joint[2] = io->body()->link("LINK_3");
        joint[3] = io->body()->link("LINK_4");
        joint[4] = io->body()->link("LINK_5");
        joint[5] = io->body()->link("LINK_6");
        
        for(int i = 0; i < 6; i++)	{
            joint[i]->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint[i]);
            q_ref[i] = q_prev[i] = joint[i]->q();
        }
        dt = io->timeStep();
        
        return true;
    }
    
    virtual bool control() override
    {
        // PD gains
        static const double P[6] = { 100.0, 100.0,  50.0, 50.0, 10.0, 10.0};
        static const double D[6] = {   1.0,   1.0,   0.5,  0.5,  0.1,  0.1};
        
        double q[6];
        double dq[6];
        double dq_ref[6];
        
        for(int i = 0; i < 6; i++)	{
            q[i]  = joint[i]->q(); // input
            dq[i] = (q[i] - q_prev[i]) / dt;
            dq_ref[i] = 0.0;
            joint[i]->u() = P[i] * (q_ref[i] - q[i]) + D[i] * (dq_ref[i] - dq[i]); // output
            q_prev[i] = q[i];
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FSCController2)
