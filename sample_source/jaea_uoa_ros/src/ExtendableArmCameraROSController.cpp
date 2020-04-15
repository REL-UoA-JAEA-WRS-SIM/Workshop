/*
    \author Fumiaki Abe
*/
#include <cnoid/SimpleController>
#include <mutex>
#include <iostream>
#include <vector>
#include <cnoid/Body>
#include <cnoid/EigenUtil>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

using namespace cnoid;
using namespace std;


class ExtendableArmCameraROSController : public cnoid::SimpleController
{
    ros::NodeHandle nh;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    vector<Link*> Links;
    sensor_msgs::Joy joystick;
    Body* body;
    double dt;

    struct JointInfo {
        Link* joint;
        double qref;
        double qold;
        double kp;
        double kd;
    };
    vector<JointInfo> jointInfos;
    Link::ActuationMode mainActuationMode;

        struct JointSpec {
        string name;
        double kp_torque;
        double kd_torque;
        double kp_velocity;
    };

    enum {
        CAM_ARM2,
        CAM_ARM3,
        CAM_ARM4,
        CAM_ARM5,
        CAM_ARM6,
        CAM_ARM7,
        NUM_JOINTS
    };
    enum ButtonID {
        A_BUTTON, // Cross
        B_BUTTON, // Circle
        X_BUTTON, // Square
        Y_BUTTON, // Triangle
        L_BUTTON,
        R_BUTTON,
        SELECT_BUTTON,
        START_BUTTON,
        L_STICK_BUTTON,
        R_STICK_BUTTON,
        LOGO_BUTTON,
        NUM_STD_BUTTONS
    };


public:

    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);

    virtual bool control() override;
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
    void joystickCallback(const sensor_msgs::Joy& msg);
};


bool ExtendableArmCameraROSController::initialize(SimpleControllerIO* io)
    {
        body = io->body();
        dt = io->timeStep();
        mainActuationMode = Link::JOINT_TORQUE;
        string prefix;
        for(auto& option : io->options()){
            if(option == "velocity"){
                mainActuationMode = Link::JOINT_VELOCITY;
            } else if(option == "position"){
                mainActuationMode = Link::JOINT_ANGLE;
            } else if(option == "torque"){
                mainActuationMode = Link::JOINT_TORQUE;
            } else {
                prefix = option;
            }
        }
        jointInfos.clear();
        const double P_GAIN_VELOCITY = 0.3;
        vector<JointSpec> specs(NUM_JOINTS);
        if(io->timeStep() < 0.02){
            //
            specs[CAM_ARM2        ] = { "CAM_ARM2",         1000.0, 100,   P_GAIN_VELOCITY };
            specs[CAM_ARM3        ] = { "CAM_ARM3",         1000.0, 100,   P_GAIN_VELOCITY };
            specs[CAM_ARM4        ] = { "CAM_ARM4",         600.0,   60,   P_GAIN_VELOCITY };
            specs[CAM_ARM5        ] = { "CAM_ARM5",         2.0,   0.15,   P_GAIN_VELOCITY };
            specs[CAM_ARM6        ] = { "CAM_ARM6",         1.5,   0.15,   P_GAIN_VELOCITY };
            specs[CAM_ARM7        ] = { "CAM_ARM7",         1.5,   0.15,   P_GAIN_VELOCITY };
        }  else{
            //
            specs[CAM_ARM2        ] = { "CAM_ARM2",         100.0,   10,   P_GAIN_VELOCITY };
            specs[CAM_ARM3        ] = { "CAM_ARM3",         100.0,   10,   P_GAIN_VELOCITY };
            specs[CAM_ARM4        ] = { "CAM_ARM4",         10.0,     6,   P_GAIN_VELOCITY };
            specs[CAM_ARM5        ] = { "CAM_ARM5",         3.0,   30,   P_GAIN_VELOCITY };
            specs[CAM_ARM6        ] = { "CAM_ARM6",         3.0,   30,   P_GAIN_VELOCITY };
            specs[CAM_ARM7        ] = { "CAM_ARM7",         3.0,   30,   P_GAIN_VELOCITY };
        }

        if(!initializeJoints(io, specs, prefix)){
            return false;
        }
        dt = io->timeStep();
        joystickSubscriber = nh.subscribe("joy", 1, &ExtendableArmCameraROSController::joystickCallback, this);

        return true;
    }


bool ExtendableArmCameraROSController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
    for(auto& spec : specs){
        string name = prefix + spec.name;
        auto joint = io->body()->link(name);
        if(!joint){
            return false;
        } 
        else{
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);

            JointInfo info;
            info.joint = joint;
            info.qref = info.qold = joint->q();
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

bool ExtendableArmCameraROSController::control()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        joystick = latestJoystickState;
        joystick.axes.resize(10, 0.0f);
        joystick.buttons.resize(13, 0);
    }
    double pos;
    //pos = joystick.getPosition(0);
    pos = joystick.axes[0]; 
    if(fabs(pos) < 0.2){
            pos = 0.0;
    }

if(joystick.buttons[L_BUTTON]){
    jointInfos[CAM_ARM5].qref += 0.0005;
}
if(joystick.buttons[R_BUTTON]){
    jointInfos[CAM_ARM5].qref -= 0.0005;
}
if(joystick.buttons[X_BUTTON]){
    jointInfos[CAM_ARM6].qref += 0.0005;
}
if(joystick.buttons[B_BUTTON]){
    jointInfos[CAM_ARM6].qref -= 0.0005;
}
if(joystick.buttons[Y_BUTTON]){
    jointInfos[CAM_ARM7].qref += 0.0005;
}
if(joystick.buttons[A_BUTTON]){
    jointInfos[CAM_ARM7].qref -= 0.0005;
}

    for(auto& info : jointInfos){
        if(info.joint->isPrismaticJoint()){
            info.qref += 0.0002*pos;
            if(info.joint->q() > 0.8){
                info.qref -= 0.0002;    
            }
            if(info.joint->q() < 0.0){
                info.qref += 0.0002;  
            }
        }
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


        return true;
    }


void ExtendableArmCameraROSController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / dt;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
}


void ExtendableArmCameraROSController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.qref - q) / dt;
    }
}


void ExtendableArmCameraROSController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.qref;
    }
}

void ExtendableArmCameraROSController::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestJoystickState = msg;    
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ExtendableArmCameraROSController)