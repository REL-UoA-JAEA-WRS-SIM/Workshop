/*
    @author Fumiaki Abe
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

class SpiderROSwith6DoFArmController : public SimpleController
{
    ros::NodeHandle nh;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;
    Body* body;
    Body* ioBody;
    Link* ioFINGER1;
    Link* ioFINGER2;
    Link* ioFINGER3;
    BodyPtr ikBody;
    Link* ikWrist;
    std::shared_ptr<cnoid::JointPath> baseToWrist;
    Link::ActuationMode mainActuationMode;
    Link::ActuationMode trackActuationMode;
    double time;
    double timeStep;
    double waitTime;

    struct JointInfo {
        Link* joint;
        double qref;
        double qold;
        double kp;
        double kd;
    };
    vector<JointInfo> jointInfos;
    vector<JointInfo> jointInfos2;
    vector<Link*> tracks;
    double trackVelocityRatio;
    double kd_mainTrack;
    double kd_subTrack;
    vector<double> qprev_track;

    struct JointSpec {
        string name;
        double kp_torque;
        double kd_torque;
        double kp_velocity;
    };


    enum { L_TRACK, R_TRACK, FL_SUB_TRACK, FR_SUB_TRACK, BL_SUB_TRACK, BR_SUB_TRACK, NUM_TRACKS };
    enum { SPACER_RF, SPACER_LF, SPACER_RR, SPACER_LR, NUM_FLIPPERS };
    //enum { SHOULDER, 
        //    ARM_HALF_1, ARM_HALF_2, FOREARM, WRIST_SPHERICAL_1, WRIST_SPHERICAL_2,
        //    HAND_3FINGER, FINGER_PROXIMAL_1, FINGER_PROXIMAL_2, FINGER_PROXIMAL_3,NUM_JOINTS};
    enum {
        SHOULDER,
                ARM,
        FOREARM,
        WRIST1,
        WRIST2,
        HAND,
        FINGER1,
        FINGER1_TIP,
        FINGER2,
        FINGER2_TIP,
        FINGER3,
        FINGER3_TIP,
        NUM_JOINTS
    };

    sensor_msgs::Joy joystick;
    SharedJoystickPtr sharedjoy;
    int targetMode;
    bool controlFlg;

public:

    Matrix3d RotateX(double radian);
    Matrix3d RotateY(double radian);
    Matrix3d RotateZ(double radian);
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeTracks(SimpleControllerIO* io);
    bool initializeTracks(SimpleControllerIO* io, vector<string>& names);
    bool initializeFlipperJoints(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs);
    bool start();
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    virtual bool control() override;
    void controlTracks();
    double deg2rad(double degree);
    double rad2deg(double radian);
    void updateTargetJointAnglesBYInverseKinematicks();
    int updateControlflg();
    void setTrackTorque(int id, double dq_target, double kd);
    void updateFlipperTargetPositions();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
    void joystickCallback(const sensor_msgs::Joy& msg);

};

Matrix3d SpiderROSwith6DoFArmController::RotateX(double radian)
{
    Matrix3d rotX = MatrixXd::Zero(3,3);
    rotX(0,0) = 1;
    rotX(1,1) = cos(radian);
    rotX(1,2) = -sin(radian);
    rotX(2,1) = sin(radian);
    rotX(2,2) = cos(radian);
    return rotX; 
}

Matrix3d SpiderROSwith6DoFArmController::RotateY(double radian)
{
    Matrix3d rotY = MatrixXd::Zero(3,3);
    rotY(0,0) = cos(radian);
    rotY(0,2) = sin(radian);
    rotY(1,1) = 1;
    rotY(2,0) = -sin(radian);
    rotY(2,2) = cos(radian);
    return rotY; 
}

Matrix3d SpiderROSwith6DoFArmController::RotateZ(double radian)
{
    Matrix3d rotZ = MatrixXd::Zero(3,3);
    rotZ(0,0) = cos(radian);
    rotZ(0,1) = -sin(radian);
    rotZ(1,0) = sin(radian);
    rotZ(1,1) = cos(radian);
    rotZ(2,2) = 1;
    return rotZ; 
}


bool SpiderROSwith6DoFArmController::initialize(SimpleControllerIO* io)
{

    body = io->body();
    ioBody = io->body();
    timeStep = io->timeStep();
    ikBody = ioBody->clone();
    ikWrist = ikBody->link("HAND");
    ioFINGER1 = ioBody->link("FINGER1");
    ioFINGER2 = ioBody->link("FINGER2");
    ioFINGER3 = ioBody->link("FINGER3");

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

    jointInfos2.clear();

    if(!initializeTracks(io)){
        return false;
    }

    if(!initializeFlipperJoints(io)){
        return false;
    }


    jointInfos.clear();

    const double P_GAIN_VELOCITY = 0.3;
    vector<JointSpec> specs(NUM_JOINTS);
    if(io->timeStep() < 0.02){
    //                                                  P      D        P (vel) 
          specs[SHOULDER    ] = { "SHOULDER",  1000.0, 100,  P_GAIN_VELOCITY };
          specs[ARM         ] = { "ARM",       1000.0, 100,  P_GAIN_VELOCITY };
          specs[FOREARM     ] = { "FOREARM",   600.0,  60,   P_GAIN_VELOCITY };
          specs[WRIST1      ] = { "WRIST1",    300.0,   30,  P_GAIN_VELOCITY };
          specs[WRIST2      ] = { "WRIST2",    300.0,   30,  P_GAIN_VELOCITY };
          specs[HAND        ] = { "HAND",      250.0,   25,  P_GAIN_VELOCITY };
          specs[FINGER1     ] = { "FINGER1",     30,     3,  P_GAIN_VELOCITY };
          specs[FINGER1_TIP ] = { "FINGER1_TIP", 20,     2,  P_GAIN_VELOCITY };
          specs[FINGER2     ] = { "FINGER2",     30,     3,  P_GAIN_VELOCITY };
          specs[FINGER2_TIP ] = { "FINGER2_TIP", 20,     2,  P_GAIN_VELOCITY };
          specs[FINGER3     ] = { "FINGER3",     30,     3,  P_GAIN_VELOCITY };
          specs[FINGER3_TIP ] = { "FINGER3_TIP", 20,     2,  P_GAIN_VELOCITY };
    } else {
    //                                                   P      D      P (vel)
          specs[SHOULDER    ] = { "SHOULDER",   400.0, 30.0,  P_GAIN_VELOCITY };
          specs[ARM         ] = { "ARM",        400.0, 30.0,  P_GAIN_VELOCITY };
          specs[FOREARM     ] = { "FOREARM",    150.0, 15.0,  P_GAIN_VELOCITY };
          specs[WRIST1      ] = { "WRIST1",      60.0,  5.0,  P_GAIN_VELOCITY };
          specs[WRIST2      ] = { "WRIST2",      60.0,  5.0,  P_GAIN_VELOCITY };
          specs[HAND        ] = { "HAND",        60.0,  5.0,  P_GAIN_VELOCITY };
          specs[FINGER1     ] = { "FINGER1",     5.0,  0.5,  P_GAIN_VELOCITY };
          specs[FINGER1_TIP ] = { "FINGER1_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
          specs[FINGER2     ] = { "FINGER2",     5.0,  0.5,  P_GAIN_VELOCITY };
          specs[FINGER2_TIP ] = { "FINGER2_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
          specs[FINGER3     ] = { "FINGER3",     5.0,  0.5,  P_GAIN_VELOCITY };
          specs[FINGER3_TIP ] = { "FINGER3_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
    }

    if(!initializeJoints(io, specs, prefix)){
        return false;
    }
    baseToWrist->calcForwardKinematics();
    time = 0.0;
    timeStep = io->timeStep();
    waitTime = io->currentTime();
    sharedjoy = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = sharedjoy->addMode();

    joystickSubscriber = nh.subscribe("joy",1, &SpiderROSwith6DoFArmController::joystickCallback,this);




    return true;
}

bool SpiderROSwith6DoFArmController::initializeTracks(SimpleControllerIO* io)
{
    tracks.clear();
    qprev_track.clear();

    vector<string> trackNames = {
        "L_TRACK", "R_TRACK", "FL_SUB_TRACK", "FR_SUB_TRACK", "BL_SUB_TRACK", "BR_SUB_TRACK" };

    vector<string> wheelNames = {
        "SPROCKET_L", "SPROCKET_R",
        "SPROCKET_LF", "SPROCKET_RF", "SPROCKET_LR", "SPROCKET_RR" };
    
    bool result;
    

    if(body->link(wheelNames[0])){
        if(mainActuationMode == Link::JOINT_TORQUE){
            trackActuationMode = Link::JOINT_TORQUE;
            trackVelocityRatio = 0.5;
            kd_mainTrack = 8.0;
            kd_subTrack = 1.5;
        } else {
            trackActuationMode= Link::JOINT_VELOCITY;
            trackVelocityRatio = 4.0;
        }
        result = initializeTracks(io, wheelNames);
    } else {
        trackActuationMode = Link::JOINT_SURFACE_VELOCITY;
        trackVelocityRatio = 0.5;
        result = initializeTracks(io, trackNames);
    }

    return result;
}

bool SpiderROSwith6DoFArmController::initializeTracks(SimpleControllerIO* io, vector<string>& names)
{
    for(auto& name : names){
        auto link = body->link(name);
        if(!link){
           // io->os() << format("{0} of {1} is not found", name, body->name()) << endl;
            return false;
        }
        link->setActuationMode( trackActuationMode);
        io->enableOutput(link);
        tracks.push_back(link);
        qprev_track.push_back(link->q());
    }
    return true;
}


bool SpiderROSwith6DoFArmController::initializeFlipperJoints(SimpleControllerIO* io)
{
    jointInfos2.clear();
    
    vector<JointSpec> specs(NUM_FLIPPERS);

    const double FLIPPER_P_GAIN_TORQUE = 800.0;
    const double FLIPPER_D_GAIN_TORQUE = 20.0;
    const double FLIPPER_P_GAIN_VELOCITY = 1.0;

    
    specs[SPACER_RF] = { "SPACER_RF", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_LF] = { "SPACER_LF", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_RR] = { "SPACER_RR", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_LR] = { "SPACER_LR", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };

    return initializeJoints(io, specs);
}


bool SpiderROSwith6DoFArmController::start()
{
    return true;
}


bool SpiderROSwith6DoFArmController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
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

bool SpiderROSwith6DoFArmController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs)
{
    for(auto& spec : specs){
        auto joint = body->link(spec.name);
        if(!joint){
            //io->os() << format("{0} of {1} is not found", spec.name, body->name()) << endl;
            return false;
        }
        joint->setActuationMode(mainActuationMode);
        io->enableIO(joint);

        JointInfo info;
        info.joint = joint;
        info.qref = info.qold = joint->q();

        if(mainActuationMode == Link::JOINT_VELOCITY){
            info.kp = spec.kp_velocity;
        } else if(mainActuationMode == Link::JOINT_TORQUE){
            info.kp = spec.kp_torque;
            info.kd = spec.kd_torque;
        }
        
        jointInfos2.push_back(info);
    }

    return true;
}


bool SpiderROSwith6DoFArmController::control()
{
       sharedjoy->updateState(targetMode);
    {
        std::lock_guard<std::mutex> lock(mutex);
        joystick = latestJoystickState;
        joystick.axes.resize(10, 0.0f);
        joystick.buttons.resize(13, 0);
    }


    if(joystick.buttons[10] )
    {
        if((time - waitTime) > 100*timeStep){
            waitTime = time;
            if(!controlFlg){controlFlg=true;}
            else{controlFlg=false;}
        }
    } 

    // if(joystick.buttons[10]){        
    //     if(!controlFlg){controlFlg=true;}
    //     else{controlFlg=false;}
    // }

    if(controlFlg){
	    updateTargetJointAnglesBYInverseKinematicks(); 
    }
    else if(!controlFlg){
        controlTracks();
        updateFlipperTargetPositions();
    }
    else{}
     
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

   
double SpiderROSwith6DoFArmController::deg2rad(double degree)
{
    return (double) (degree * M_PI / 180.0);
}

double SpiderROSwith6DoFArmController::rad2deg(double radian)
{
    return (double) (radian * 180.0/ M_PI);
}

void SpiderROSwith6DoFArmController::updateTargetJointAnglesBYInverseKinematicks()
{ 
        // 手首座標
        baseToWrist->calcForwardKinematics();
        Vector3d p =  ikWrist->p();
        Matrix3d R =  ikWrist->R();
        p(1) -= joystick.axes[0]/500;
        p(0) -= joystick.axes[1]/500;
        p(2) -= joystick.axes[3]/500;

        if(joystick.buttons[4]){
            R *= RotateX(deg2rad(-0.3));
        }
        if(joystick.buttons[5]){
            R *= RotateX(deg2rad(0.3));
        }
        // 手首の操作
        if(joystick.buttons[0]){
            R *= RotateZ(deg2rad(-0.3)) ;
        }
        if(joystick.buttons[3]){
            R *= RotateZ(deg2rad(0.3));
        }
        if(joystick.buttons[2]){
            R *= RotateY(deg2rad(0.3));
        }
        if(joystick.buttons[1]){
            R *= RotateY(deg2rad(-0.3));
        }

        baseToWrist->calcInverseKinematics(p, R);

        Vector3d rpy = rpyFromRot(ikWrist->attitude());
         for(int i = 0; i <6; i++ )
         {
            Link* joint = baseToWrist->joint(i);             
	        jointInfos[i].qref = joint->q();	
         }
        double dq_fingerL = 0.0;
        double ltL = joystick.axes[6];
        dq_fingerL -= ltL;
        //double 
        ltL = joystick.axes[7];
        dq_fingerL += ltL;
    
        for(int i = FINGER1; i <= FINGER3; ++i){
            jointInfos[i].qref += 0.003*dq_fingerL;
        }
}

void SpiderROSwith6DoFArmController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / timeStep;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
        for(auto& info : jointInfos2){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / timeStep;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
}


void SpiderROSwith6DoFArmController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.qref - q) / timeStep;
    }
        for(auto& info : jointInfos2){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.qref - q) / timeStep;
    }
}


void SpiderROSwith6DoFArmController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.qref;
    }
        for(auto& info : jointInfos2){
        info.joint->q_target() = info.qref;
    }
}

void SpiderROSwith6DoFArmController::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestJoystickState = msg;    
}

void SpiderROSwith6DoFArmController::controlTracks()
{
    double hpos = joystick.axes[0];
    double vpos = -joystick.axes[1];
    
    double dq_L = trackVelocityRatio * (vpos + 0.4 * hpos);
    double dq_R = trackVelocityRatio * (vpos - 0.4 * hpos);

    switch(trackActuationMode){

    case Link::JOINT_VELOCITY:
    case Link::JOINT_SURFACE_VELOCITY:
        for(int i=0; i < 3; ++i){
            tracks[i*2  ]->dq_target() = dq_L;
            tracks[i*2+1]->dq_target() = dq_R;
        }
        break;

    case Link::JOINT_TORQUE:
        setTrackTorque(L_TRACK, dq_L, kd_mainTrack);
        setTrackTorque(R_TRACK, dq_R, kd_mainTrack);
        setTrackTorque(FL_SUB_TRACK, dq_L, kd_subTrack);
        setTrackTorque(FR_SUB_TRACK, dq_R, kd_subTrack);
        setTrackTorque(BL_SUB_TRACK, dq_L, kd_subTrack);
        setTrackTorque(BR_SUB_TRACK, dq_R, kd_subTrack);
        break;

    default:
        break;
    }
}


void SpiderROSwith6DoFArmController::setTrackTorque(int id, double dq_target, double kd)
{
    Link* axis = tracks[id];
    double dq_current = (axis->q() - qprev_track[id]) / timeStep;
    axis->u() = kd * (dq_target - dq_current);
    qprev_track[id] = axis->q();
}


void SpiderROSwith6DoFArmController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.1;

    if(joystick.buttons[12]){
        double qa = 0.0;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            qa += jointInfos2[i].qref;
        }
        qa /= NUM_FLIPPERS;
        double dqmax = timeStep * 0.5;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            double dq = qa - jointInfos2[i].qref;
            if(dq > dqmax){
                dq = dqmax;
            } else if(dq < -dqmax){
                dq = -dqmax;
            }
            jointInfos2[i].qref += dq;
        }
    } else {
        double pos = joystick.axes[3] * 5.0;
        double dq = timeStep * FLIPPER_GAIN * pos;
        bool FL = joystick.buttons[4];
        bool FR = joystick.buttons[5];
        bool BL = false;
        bool BR = false;


        if(joystick.axes[6] > 0){BL = true;} 
        else {BL = false;}
        if(joystick.axes[7] > 0){BR = true;} 
        else {BR = false;}

        if(!FL && !FR && !BL && !BR){
            // Synchronize mode
            jointInfos2[SPACER_RF].qref -= dq;
            jointInfos2[SPACER_LF].qref -= dq;
            jointInfos2[SPACER_RR].qref += dq;
            jointInfos2[SPACER_LR].qref += dq;
        } else {
            if(FL){
                jointInfos2[SPACER_LF].qref -= dq;
            }
            if(FR){
                jointInfos2[SPACER_RF].qref -= dq;
            }
            if(BL){
                jointInfos2[SPACER_LR].qref += dq;
            }
            if(BR){
                jointInfos2[SPACER_RR].qref += dq;
            }
        }
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpiderROSwith6DoFArmController)
