/*
  \author Fumiaki Abe
*/
#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <mutex>
#include <iostream>
#include <vector>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace cnoid;

class Jaco2IKController7DOF_ROS : public SimpleController
{
    ros::NodeHandle nh;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    
    Body* ioBody;
    Link* ioFINGER1;
    Link* ioFINGER2;
    Link* ioFINGER3;
    BodyPtr ikBody;
    Link* ikWrist;
    std::shared_ptr<cnoid::JointPath> baseToWrist;
    Link::ActuationMode mainActuationMode;
    double time;
    double timeStep;

    struct JointInfo {
        Link* joint;
        double q_ref;
        double q_old;
        double kp;
        double kd;
    };

    vector<JointInfo> jointInfos;

    struct JointSpec {
        string name;
        double kp_torque;
        double kd_torque;
        double kp_velocity;
    };

    enum {
        SHOULDER,
        ARM_HALF_1,
        ARM_HALF_2,
        FOREARM,
        WRIST_SPHERICAL_1,
        WRIST_SPHERICAL_2,
        HAND_3FINGER,
        FINGER_PROXIMAL_1,
        FINGER_PROXIMAL_2,
        FINGER_PROXIMAL_3,
        NUM_JOINTS
    };

    sensor_msgs::Joy joystick;
    
    int targetMode;
    bool controlFlg;

public:

    Matrix3d RotateX(double radian);
    Matrix3d RotateY(double radian);
    Matrix3d RotateZ(double radian);
    virtual bool initialize(SimpleControllerIO* io) override;
    bool start();
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    virtual bool control() override;
    double deg2rad(double degree);
    double rad2deg(double radian);
    void updateTargetJointAnglesBYInverseKinematicks();
    int updateControlflg();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
    void joystickCallback(const sensor_msgs::Joy& msg);

};

Matrix3d Jaco2IKController7DOF_ROS::RotateX(double radian)
{
    Matrix3d rotX = MatrixXd::Zero(3,3);
    rotX(0,0) = 1;
    rotX(1,1) = cos(radian);
    rotX(1,2) = -sin(radian);
    rotX(2,1) = sin(radian);
    rotX(2,2) = cos(radian);
    return rotX; 
}

Matrix3d Jaco2IKController7DOF_ROS::RotateY(double radian)
{
    Matrix3d rotY = MatrixXd::Zero(3,3);
    rotY(0,0) = cos(radian);
    rotY(0,2) = sin(radian);
    rotY(1,1) = 1;
    rotY(2,0) = -sin(radian);
    rotY(2,2) = cos(radian);
    return rotY; 
}

Matrix3d Jaco2IKController7DOF_ROS::RotateZ(double radian)
{
    Matrix3d rotZ = MatrixXd::Zero(3,3);
    rotZ(0,0) = cos(radian);
    rotZ(0,1) = -sin(radian);
    rotZ(1,0) = sin(radian);
    rotZ(1,1) = cos(radian);
    rotZ(2,2) = 1;
    return rotZ; 
}


bool Jaco2IKController7DOF_ROS::initialize(SimpleControllerIO* io)
{
    ioBody = io->body();
    timeStep = io->timeStep();
    ikBody = ioBody->clone();
    ikWrist = ikBody->link("HAND_3FINGER");
    ioFINGER1 = ioBody->link("FINGER_PROXIMAL_1");
    ioFINGER2 = ioBody->link("FINGER_PROXIMAL_2");
    ioFINGER3 = ioBody->link("FINGER_PROXIMAL_3");

    Link* base = ikBody->rootLink();
    baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
    base->p().setZero();
    base->R().setIdentity(); 
    mainActuationMode = Link::JOINT_TORQUE;
    string prefix;
    for(auto& option : io->options()){
        if(option == "velocity"){
            mainActuationMode = Link::JOINT_VELOCITY;
            io->os() << "velocity mode" << endl;
        } else if(option == "position"){
            mainActuationMode = Link::JOINT_ANGLE;
            io->os() << "position mode" << endl;
        } else if(option == "torque"){
            mainActuationMode = Link::JOINT_TORQUE;
            io->os() << "torque mode" << endl;
        } else {
            prefix = option;
            io->os() << "prefix: " << prefix << endl;
        }
    }

    jointInfos.clear();
    const double P_GAIN_VELOCITY = 0.3;
    vector<JointSpec> specs(NUM_JOINTS);
    if(io->timeStep() < 0.02){
    //                                                  P      D        P (vel) 
    specs[SHOULDER        ] = { "SHOULDER",           1000.0, 100,   P_GAIN_VELOCITY };
    specs[ARM_HALF_1       ] = { "ARM_HALF_1",         1000.0, 100,   P_GAIN_VELOCITY };
    specs[ARM_HALF_2       ] = { "ARM_HALF_2",         600.0,   60,   P_GAIN_VELOCITY };
    specs[FOREARM          ] = { "FOREARM",            600.0,   60,   P_GAIN_VELOCITY };
    specs[WRIST_SPHERICAL_1] = { "WRIST_SPHERICAL_1",  300.0,   30,   P_GAIN_VELOCITY };
    specs[WRIST_SPHERICAL_2] = { "WRIST_SPHERICAL_2",  300.0,   30,   P_GAIN_VELOCITY };
    specs[HAND_3FINGER     ] = { "HAND_3FINGER",       250.0,   25,   P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_1] = { "FINGER_PROXIMAL_1",     5,     3,  P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_2] = { "FINGER_PROXIMAL_2",     5,     3,  P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_3] = { "FINGER_PROXIMAL_3",     5,     3,  P_GAIN_VELOCITY };
    } else {
    //                                                   P      D      P (vel)
    specs[SHOULDER         ] = { "SHOULDER",           400.0,   30,   P_GAIN_VELOCITY };
    specs[ARM_HALF_1       ] = { "ARM_HALF_1",         400.0,   30,   P_GAIN_VELOCITY };
    specs[ARM_HALF_2       ] = { "ARM_HALF_2",         150.0,   15,   P_GAIN_VELOCITY };
    specs[FOREARM          ] = { "FOREARM",            150.0,   15,   P_GAIN_VELOCITY };
    specs[WRIST_SPHERICAL_1] = { "WRIST_SPHERICAL_1",   60.0,    5,   P_GAIN_VELOCITY };
    specs[WRIST_SPHERICAL_2] = { "WRIST_SPHERICAL_2",   60.0,    5,   P_GAIN_VELOCITY };
    specs[HAND_3FINGER     ] = { "HAND_3FINGER",        60.0,    5,   P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_1] = { "FINGER_PROXIMAL_1",      5,  0.5,  P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_2] = { "FINGER_PROXIMAL_2",      5,  0.5,  P_GAIN_VELOCITY };
    specs[FINGER_PROXIMAL_3] = { "FINGER_PROXIMAL_3",      5,  0.5,  P_GAIN_VELOCITY };
    }

    if(!initializeJoints(io, specs, prefix)){
        return false;
    }
    baseToWrist->calcForwardKinematics();
    time = 0.0;
    timeStep = io->timeStep();
    //joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    //targetMode = joystick->addMode();

    joystickSubscriber = nh.subscribe("joy",1, &Jaco2IKController7DOF_ROS::joystickCallback,this);

    return true;
}

bool Jaco2IKController7DOF_ROS::start()
{
    return true;
}


bool Jaco2IKController7DOF_ROS::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
    for(auto& spec : specs){
        string name = prefix + spec.name;
        auto joint = ioBody->link(name);
        if(!joint){
            return false;
        } 
        else{
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);

            JointInfo info;
            info.joint = joint;
            info.q_ref = info.q_old = joint->q();
            if(mainActuationMode == Link::JOINT_VELOCITY){
                info.kp = spec.kp_velocity;
            } 
            else if(mainActuationMode == Link::JOINT_TORQUE){
                info.kp = spec.kp_torque;
                info.kd = spec.kd_torque;
            }    
            jointInfos.push_back(info);
        } 
    }  
    return true;
}

bool Jaco2IKController7DOF_ROS::control()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        joystick = latestJoystickState;
        joystick.axes.resize(10, 0.0f);
        joystick.buttons.resize(13, 0);
    }

    if(joystick.buttons[10]){        
        if(!controlFlg){controlFlg=true;}
        else{controlFlg=false;}
    }
    if(!controlFlg){
	    updateTargetJointAnglesBYInverseKinematicks(); 
    }
     
        switch(mainActuationMode){
        case Link::JOINT_TORQUE:
            controlJointsWithTorque();
            break;
        case Link::JOINT_VELOCITY:
            controlJointsWithVelocity();
            break;
        case Link::JOINT_ANGLE:
            controlJointsWithPosition();
            break;
        default:
            break;
        }

    time += timeStep;
    return true;
}

   
double Jaco2IKController7DOF_ROS::deg2rad(double degree)
{
    return (double) (degree * M_PI / 180.0);
}

double Jaco2IKController7DOF_ROS::rad2deg(double radian)
{
    return (double) (radian * 180.0/ M_PI);
}

void Jaco2IKController7DOF_ROS::updateTargetJointAnglesBYInverseKinematicks()
{ 
        // 手首座標
        baseToWrist->calcForwardKinematics();
        Vector3d p =  ikWrist->p();
        Matrix3d R =  ikWrist->R();
        p(1) -= joystick.axes[0]/200;
        p(0) -= joystick.axes[1]/200;
        p(2) -= joystick.axes[3]/200;

        if(joystick.buttons[5]){
            R *= RotateX(deg2rad(-1));
        }
        if(joystick.buttons[4]){
            R *= RotateX(deg2rad(1));
        }
        // 手首の操作
        if(joystick.buttons[0]){
            R *= RotateZ(deg2rad(-1)) ;
        }
        if(joystick.buttons[3]){
            R *= RotateZ(deg2rad(1));
        }
        if(joystick.buttons[1]){
            R *= RotateY(deg2rad(1));
        }
        if(joystick.buttons[2]){
            R *= RotateY(deg2rad(-1));
        }

        baseToWrist->calcInverseKinematics(p, R);
        

        Vector3d rpy = rpyFromRot(ikWrist->attitude());
         for(int i = 0; i <7; i++ )
         {
            Link* joint = baseToWrist->joint(i);             
	        jointInfos[i].q_ref = joint->q();	
         }
        double dq_fingerL = 0.0;
        double ltL = joystick.axes[6];
        dq_fingerL -= ltL;
        //double 
        ltL = joystick.axes[7];
        dq_fingerL += ltL;
    
        for(int i = FINGER_PROXIMAL_1; i <= FINGER_PROXIMAL_3; ++i){
            jointInfos[i].q_ref += 0.003*dq_fingerL;
        }
}

void Jaco2IKController7DOF_ROS::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.q_old) / timeStep;
        joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
        info.q_old = q;
    }
}


void Jaco2IKController7DOF_ROS::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.q_ref - q) / timeStep;
    }
}


void Jaco2IKController7DOF_ROS::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.q_ref;
    }
}

void Jaco2IKController7DOF_ROS::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestJoystickState = msg;    
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaco2IKController7DOF_ROS)
