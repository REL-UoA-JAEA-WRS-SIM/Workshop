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
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace cnoid;

class EnryuIKROSEndscopeController : public SimpleController
{
public:
    ros::NodeHandle nh;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;
    sensor_msgs::Joy joystick;

    enum TrackType { NO_TRACKS = 0, CONTINOUS_TRACKS, PSEUDO_TRACKS };
    int trackType;
    Link* trackL;
    Link* trackR;
    double trackgain;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> q_ref;
    vector<double> q_prev;
    vector<double> pgain;
    vector<double> dgain;
    double* q_tip1;
    double* q_tip2;
    

    Link::ActuationMode mainActuationMode;
    Body* body;
    Body* ikBody;
    Link* ikWrist;
    Link* ikWrist2;
    std::shared_ptr<cnoid::JointPath> baseToWrist;
    std::shared_ptr<cnoid::JointPath> baseToWrist2;
    double dt;
    double time;
    double waitTime;

    int arm1Mode;
    int arm2Mode;
    int currentJoystickMode;
    const int SHIFT_BUTTON = Joystick::L_BUTTON;
    int shiftState;

    enum AxisType { STICK, BUTTON };
    enum AxisID {
        L_STICK_H_AXIS,
        L_STICK_V_AXIS,
        R_STICK_H_AXIS,
        R_STICK_V_AXIS,
        DIRECTIONAL_PAD_H_AXIS,
        DIRECTIONAL_PAD_V_AXIS,
        L_TRIGGER_AXIS,
        R_TRIGGER_AXIS,
        NUM_STD_AXES
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

    struct OperationAxis {
        Link* joint;
        AxisType type;
        int id;
        double ratio;
        int shift;
        OperationAxis(Link* joint, AxisType type, int id, double ratio, int shift = 0)
            : joint(joint), type(type), id(id), ratio(ratio), shift(shift) { }
    };

    
    vector<vector<OperationAxis>> operationAxes;
    int operationSetIndex;


    struct JointInfo {
        Link* joint;
        double q_ref;
        double q_old;
        double kp;
        double kd;
    };

    vector<JointInfo> jointInfos;
    vector<JointInfo> jointInfos2;
    struct JointSpec {
        string name;
        double kp_torque;
        double kd_torque;
        double kp_velocity;
    };
    enum {
        MFRAME,
        BLOCK,
        BOOM,
        ARM,
        TOHKU_PITCH,
        TOHKU_ROLL,
        TOHKU_TIP_01,
        TOHKU_TIP_02,
        MNP_SWING,
        MANIBOOM,
        MANIARM,
        MANIELBOW,
        YAWJOINT,
        HANDBASE,
        PUSHROD,
        CAM_ARM2,
        CAM_ARM3,
        CAM_ARM4,
        A,
        B,
        C,
        D,
        E,
        NUM_JOINTS
    };


public:
    EnryuIKROSEndscopeController();

    virtual bool initialize(SimpleControllerIO* io) override;
    bool initContinuousTracks(SimpleControllerIO* io);
    bool initPseudoContinuousTracks(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    void initArms(SimpleControllerIO* io);
    void initPDGain();
    void initJoystickKeyBind();
    bool start();
    void controlTracks();
    void setTargetArmPositions();
    void controlArms();
    void controlArmsWithTorque();
    void controlArmsWithVelocity();
    void controlArmsWithPosition();    
    double deg2rad(double degree);
    double rad2deg(double radian);
    Matrix3d RotateX(double radian);
    Matrix3d RotateY(double radian);
    Matrix3d RotateZ(double radian);
    void setArmIK();

    virtual bool control() override;
    void joystickCallback(const sensor_msgs::Joy& msg);

    Link* link(const char* name) { return body->link(name); }
};

double EnryuIKROSEndscopeController::deg2rad(double degree)
{
    return (double) (degree * M_PI / 180.0);
}

double EnryuIKROSEndscopeController::rad2deg(double radian)
{
    return (double) (radian * 180.0/ M_PI);
}
Matrix3d EnryuIKROSEndscopeController::RotateX(double radian)
{
    Matrix3d rotX = MatrixXd::Zero(3,3);
    rotX(0,0) = 1;
    rotX(1,1) = cos(radian);
    rotX(1,2) = -sin(radian);
    rotX(2,1) = sin(radian);
    rotX(2,2) = cos(radian);
    return rotX; 
}

Matrix3d EnryuIKROSEndscopeController::RotateY(double radian)
{
    Matrix3d rotY = MatrixXd::Zero(3,3);
    rotY(0,0) = cos(radian);
    rotY(0,2) = sin(radian);
    rotY(1,1) = 1;
    rotY(2,0) = -sin(radian);
    rotY(2,2) = cos(radian);
    return rotY; 
}

Matrix3d EnryuIKROSEndscopeController::RotateZ(double radian)
{
    Matrix3d rotZ = MatrixXd::Zero(3,3);
    rotZ(0,0) = cos(radian);
    rotZ(0,1) = -sin(radian);
    rotZ(1,0) = sin(radian);
    rotZ(1,1) = cos(radian);
    rotZ(2,2) = 1;
    return rotZ; 
}

EnryuIKROSEndscopeController::EnryuIKROSEndscopeController()
{
    mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
    trackType = NO_TRACKS;
}


bool EnryuIKROSEndscopeController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    ikBody = body->clone();
    ikWrist = ikBody->link("TOHKU_ROLL");
    ikWrist2 = ikBody->link("HANDBASE");
    Link* base = ikBody->link("MFRAME");
    base->p().setZero();
    base->R().setIdentity();
    baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
    baseToWrist2 = getCustomJointPath(ikBody, base, ikWrist2);
    dt = io->timeStep();
    waitTime = io->currentTime();


    // 全関節のinitialize
    string option = io->optionString();
    mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
    string prefix;
    option = "velocity";
    if(option == "velocity"){
        mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
    } else if(option  == "position"){
        mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
    } else {
        mainActuationMode = Link::ActuationMode::JOINT_EFFORT;
        prefix = option;
    }


    jointInfos.clear();
    const double P_GAIN_VELOCITY = 0.3;
    vector<JointSpec> specs(NUM_JOINTS);

 if(io->timeStep() < 0.02){
    //                                          P          D           P (vel) 
    specs[MFRAME      ] = { "MFRAME",         200000.0,  20000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",          150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",           150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",            100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",     30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",      20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",      500.0,     50.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",      500.0,     50.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",       50000.0,   5000.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",       30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",        20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",          500.0,     50.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",         50000.0,   5000.0,   P_GAIN_VELOCITY };
    specs[CAM_ARM2    ] = { "CAM_ARM2",         1000.0,      100,   P_GAIN_VELOCITY };
    specs[CAM_ARM3    ] = { "CAM_ARM3",         1000.0,      100,   P_GAIN_VELOCITY };
    specs[CAM_ARM4    ] = { "CAM_ARM4",          600.0,       60,   P_GAIN_VELOCITY };
    specs[A           ] = { "A",                   0.6,     0.07,   P_GAIN_VELOCITY };
    specs[B           ] = { "B",                   3.8,     0.08,   P_GAIN_VELOCITY };
    specs[C           ] = { "C",                   2.4,     0.03,   P_GAIN_VELOCITY };
    specs[D           ] = { "D",                   2.2,     0.03,   P_GAIN_VELOCITY };
    specs[E           ] = { "E",                   2.0,    0.005,   P_GAIN_VELOCITY };
    
    
    } else {
    //                                          P      D      P (vel)
    specs[MFRAME      ] = { "MFRAME",         20000.0,  2000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",          15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",           15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",            10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",     3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",      2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",      50.0,     5.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",      50.0,     5.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",       5000.0,   500.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",       3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",        2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",          50.0,     5.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",         5000.0,   500.0,   P_GAIN_VELOCITY };
    specs[CAM_ARM2    ] = { "CAM_ARM2",         100.0,      10,   P_GAIN_VELOCITY };
    specs[CAM_ARM3    ] = { "CAM_ARM3",         100.0,      10,   P_GAIN_VELOCITY };
    specs[CAM_ARM4    ] = { "CAM_ARM4",          60.0,       6,   P_GAIN_VELOCITY };
    specs[A           ] = { "A",                  0.6,    0.07,   P_GAIN_VELOCITY };
    specs[B           ] = { "B",                  3.8,    0.08,   P_GAIN_VELOCITY };
    specs[C           ] = { "C",                  2.4,    0.03,   P_GAIN_VELOCITY };
    specs[D           ] = { "D",                  2.2,    0.03,   P_GAIN_VELOCITY };
    specs[E           ] = { "E",                  2.0,   0.005,   P_GAIN_VELOCITY };
    }

    if(!initializeJoints(io, specs, prefix)){
        return false;
    }
   
    initContinuousTracks(io) || initPseudoContinuousTracks(io);
    initJoystickKeyBind();
    shiftState = 1;

    joystickSubscriber = nh.subscribe("joy",1, &EnryuIKROSEndscopeController::joystickCallback,this);

    return true;
}

bool EnryuIKROSEndscopeController::start()
{
    return true;
}


bool EnryuIKROSEndscopeController::initContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("WHEEL_L0");
    trackR = link("WHEEL_R0");

    if(!trackL || !trackR){
        return false;
    }
        trackL->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);
        trackR->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);

    io->enableOutput(trackL);
    io->enableOutput(trackR);
    trackType = CONTINOUS_TRACKS;
    io->os() << "Continuous tracks of " << body->name() << " are detected." << endl;    

    return true;
}

bool EnryuIKROSEndscopeController::initPseudoContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("TRACK_L");
    trackR = link("TRACK_R");
    if(!trackL || !trackR){
        return false;
    }

    if(trackL->actuationMode() == Link::JOINT_SURFACE_VELOCITY && trackR->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        trackType = PSEUDO_TRACKS;
        io->os() << "Pseudo continuous tracks of " << body->name() << " are detected." << endl;
    }

    return (trackType == PSEUDO_TRACKS);
}


bool EnryuIKROSEndscopeController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
    for(auto& spec : specs){
        string name = prefix + spec.name;
        auto joint = body->link(name);
        if(!joint){
            io->os() << "test2 : " << name << endl;
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


void EnryuIKROSEndscopeController::initJoystickKeyBind()
{
    operationAxes = {
        {
            { link("MFRAME"),       STICK,  L_STICK_H_AXIS, -0.6 },
            { link("BLOCK"),        STICK,  R_STICK_H_AXIS, -0.6 },
            { link("BOOM"),         STICK,  L_STICK_V_AXIS, -0.6 },
            { link("ARM"),          STICK,  R_STICK_V_AXIS,  0.6 },
            { link("TOHKU_PITCH"),  BUTTON, A_BUTTON,        0.6 },
            { link("TOHKU_PITCH"),  BUTTON, Y_BUTTON,       -0.6 },
            { link("TOHKU_ROLL"),   BUTTON, X_BUTTON,        1.0 },
            { link("TOHKU_ROLL"),   BUTTON, B_BUTTON,       -1.0 },
            { link("TOHKU_TIP_01"), STICK,  R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_02"), STICK,  R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_01"), BUTTON, R_BUTTON,        0.5 },
            { link("TOHKU_TIP_02"), BUTTON, R_BUTTON,        0.5 }
        },
        {
            { link("MNP_SWING"),    STICK,  R_STICK_H_AXIS, -0.6    },
            { link("MANIBOOM"),     STICK,  L_STICK_V_AXIS, -0.6    },
            { link("MANIARM"),      STICK,  R_STICK_V_AXIS,  0.6    },
            { link("MANIELBOW"),   BUTTON,       A_BUTTON,   0.6    },
            { link("MANIELBOW"),   BUTTON,       Y_BUTTON,  -0.6    },
            { link("YAWJOINT"),    BUTTON,       X_BUTTON,   1.0, 1 },
            { link("YAWJOINT"),    BUTTON,       B_BUTTON,  -1.0, 1 },
            { link("HANDBASE"),    BUTTON,       X_BUTTON,  -1.0, 0 },
            { link("HANDBASE"),    BUTTON,       B_BUTTON,   1.0, 0 },
            { link("PUSHROD"),      STICK, R_TRIGGER_AXIS,  -0.04   },
            { link("PUSHROD"),     BUTTON,       R_BUTTON,   0.04   },
        },
        {
            { link("CAM_ARM2"),     STICK,  R_TRIGGER_AXIS,  -0.01   },
            { link("CAM_ARM3"),     STICK,  R_TRIGGER_AXIS,  -0.01   },
            { link("CAM_ARM4"),     STICK,  R_TRIGGER_AXIS,  -0.01   },
            { link("CAM_ARM2"),    BUTTON,        R_BUTTON,   0.05   },
            { link("CAM_ARM3"),    BUTTON,        R_BUTTON,   0.05   },
            { link("CAM_ARM4"),    BUTTON,        R_BUTTON,   0.05   },
            { link("A"),            STICK,   L_STICK_H_AXIS,  0.6   },
            { link("B"),            STICK,   L_STICK_V_AXIS, -0.6   },
            { link("C"),            STICK,   R_STICK_V_AXIS, -0.6   },
            { link("D"),           BUTTON,         A_BUTTON, -0.6   },
            { link("D"),           BUTTON,         Y_BUTTON,  0.6   },
            { link("E"),           BUTTON,         L_BUTTON,  0.6   },
            { link("E"),            STICK,   L_TRIGGER_AXIS, -0.6   },
        }
    };

    operationSetIndex = 0;
}

void EnryuIKROSEndscopeController::controlTracks()
{
    trackL->u() = 0.0;
    trackL->dq() = 0.0;
    trackR->u() = 0.0;
    trackR->dq() = 0.0;
    
    const double k1 = 0.2;
    const double k2 = 0.4;

    double pos[2];
    pos[0] =  -joystick.axes[DIRECTIONAL_PAD_H_AXIS]; // 十字キー入力
    pos[1] =   joystick.axes[DIRECTIONAL_PAD_V_AXIS]; // 十字キー入力

    if(trackType == CONTINOUS_TRACKS && mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
        trackL->u() = trackgain * (-1.0 * pos[1] + pos[0]);
        trackR->u() = trackgain * (-1.0 * pos[1] - pos[0]);
    } else {
        trackL->dq_target() = (-2.0 * pos[1] - pos[0]);
        trackR->dq_target() = (-2.0 * pos[1] + pos[0]);
    }
}


bool EnryuIKROSEndscopeController::control()
{
    {
        std::lock_guard<std::mutex> lock(joystickMutex);
        joystick = latestJoystickState;
        joystick.axes.resize(10, 0.0f);
        joystick.buttons.resize(13, 0);
    }
    if(joystick.buttons[LOGO_BUTTON] )
    {
        if((time - waitTime) > 100*dt){
            waitTime = time;
            operationSetIndex++;
            if(operationSetIndex >= 3){
                operationSetIndex = 0;
            }
        }
    }    
    if(trackType){
        controlTracks();
    }
    controlArms();
    time += dt;
    return true;
}


void EnryuIKROSEndscopeController::setTargetArmPositions()
{
    const vector<OperationAxis>& axes = operationAxes[operationSetIndex];

    for(auto& axis : axes){
        if(axis.shift < 0 || axis.shift == shiftState){
        auto joint = axis.joint;
        auto& q = jointInfos[joint->jointId()].q_ref;
            if(axis.type == BUTTON){
                if(joystick.buttons[axis.id]){
                    q += axis.ratio * dt;
                }
            } else if(axis.type == STICK){
                  auto pos = joystick.axes[axis.id];
                  q += axis.ratio * pos * dt;
            }
        }
    }
}


void EnryuIKROSEndscopeController::setArmIK()
{ 
    // 手首座標
    ikBody = body->clone();
    ikWrist = ikBody->link("TOHKU_ROLL");
    ikWrist2 = ikBody->link("HANDBASE");
    Link* base = ikBody->link("MFRAME");
    base->p().setZero();
    base->R().setIdentity();
    baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
    baseToWrist2 = getCustomJointPath(ikBody, base, ikWrist2);
    baseToWrist->calcForwardKinematics();
    baseToWrist2->calcForwardKinematics();

    Vector3d p =  ikWrist->p();
    Matrix3d R =  ikWrist->R();
    Vector3d p2 =  ikWrist2->p();
    Matrix3d R2 =  ikWrist2->R();
    double dq_fingerL = 0.0;
    double ltL = 0.0;
    Link* TohkuTipJoint = body->link("TOHKU_TIP_02");
    Link* Pushrod = body->link("PUSHROD");
    double maxerror;
    double q_current;
    double q_lower;
    double q_upper;
    double q_lower2;
    double q_upper2;
    double dAngle;
    double baseDRot;

    switch(operationSetIndex){
    case 0:
        p(1) -= joystick.axes[0]/200;
        p(0) -= joystick.axes[1]/200; 
        p(2) -= joystick.axes[3]/200;
        if(joystick.buttons[1]) { R *= RotateX(deg2rad(1)); }
        if(joystick.buttons[2]) { R *= RotateX(deg2rad(-1));  }
        if(joystick.buttons[0]) { R *= RotateY(deg2rad(1));  }
        if(joystick.buttons[3]) { R *= RotateY(deg2rad(-1)); }
            
        // **Control of TOHKU_TIP
        ltL = joystick.axes[7];
        dq_fingerL -= ltL;
        ltL = joystick.buttons[5];
        dq_fingerL += ltL;
        if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
            maxerror = 20.0;
        } else {
            maxerror = 0.01;
        }
        q_current = TO_DEGREE*TohkuTipJoint->q();
        q_lower = -40.0;
        q_upper = 0.0;
        q_lower2 = std::max(q_current - maxerror, q_lower);
        q_upper2 = std::min(q_current + maxerror, q_upper);
        if(q_current < q_lower2){
            dq_fingerL = 0.1;
        }
        if(q_current > q_upper2){
            dq_fingerL = -0.1;
        }
        jointInfos[TohkuTipJoint->jointId()-1].q_ref += 0.005*dq_fingerL;
        jointInfos[TohkuTipJoint->jointId()  ].q_ref += 0.005*dq_fingerL;
        // **Control of MFRAME
        dAngle = joystick.axes[6];
        baseDRot += dAngle;
        dAngle = joystick.buttons[4];
        baseDRot -= dAngle;
        jointInfos[base->jointId()].q_ref += 0.01*baseDRot;
        break;

    case 1:
        p2(1) -= joystick.axes[0]/200;
        p2(0) -= joystick.axes[1]/200; 
        p2(2) -= joystick.axes[3]/200;
        if(joystick.buttons[1]){ R2 *= RotateZ(deg2rad(-1)); }
        if(joystick.buttons[2]){ R2 *= RotateZ(deg2rad(1));  }
        if(joystick.buttons[0]){ R2 *= RotateY(deg2rad(1));  }
        if(joystick.buttons[3]){ R2 *= RotateY(deg2rad(-1)); }
        if(joystick.axes[6])   { R2 *= RotateX(deg2rad(-1)); }
        if(joystick.buttons[4]){ R2 *= RotateX(deg2rad(1)); } 


        // **Control of PUSHROD
        ltL = joystick.axes[7];
        dq_fingerL -= ltL;
        ltL = joystick.buttons[5];
        dq_fingerL += ltL;
        if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
            maxerror = 20.0;
        } else {
            maxerror = 0.01;
        }
        q_current = TO_DEGREE*TohkuTipJoint->q();
        q_lower = -0.05;
        q_upper = 0.0;
        q_lower2 = std::max(q_current - maxerror, q_lower);
        q_upper2 = std::min(q_current + maxerror, q_upper);
        if(q_current < q_lower2){
            dq_fingerL = 0.1;
        }
        if(q_current > q_upper2){
            dq_fingerL = -0.1;
        }
        jointInfos[Pushrod->jointId()].q_ref += 0.0008*dq_fingerL;

        
        break;
    default:
        break;
    }

    baseToWrist->calcInverseKinematics(p, R);
    for(int i = 0; i <5; i++ )
    {
       Link* joint = baseToWrist->joint(i);
	   jointInfos[joint->jointId()].q_ref = joint->q();	
    }

    baseToWrist2->calcInverseKinematics(p2, R2);
    for(int i = 0; i <6; i++ )
    {
       Link* joint = baseToWrist2->joint(i);
	   jointInfos[joint->jointId()].q_ref = joint->q();	
    }
}

void EnryuIKROSEndscopeController::controlArms()
{
    if(joystick.buttons[7] )
    {
        if((time - waitTime) > 100*dt){
            waitTime = time;
            shiftState++;
            if(shiftState >= 2){
                shiftState = 0;
            }
        }
    }
    for(auto& info : jointInfos){
        Link* joint = info.joint;
        info.q_ref = joint->q();
    }
    if(shiftState == 1){setArmIK();}
    else{setTargetArmPositions();}

    switch(mainActuationMode){
    case Link::ActuationMode::JOINT_DISPLACEMENT:
        controlArmsWithPosition();
        break;
    case Link::ActuationMode::JOINT_VELOCITY:
        controlArmsWithVelocity();
        break;
    case Link::ActuationMode::JOINT_EFFORT:
         controlArmsWithTorque();
         break;
    default:
        break;
    }
}


void EnryuIKROSEndscopeController::controlArmsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.q_ref;
    }
}


void EnryuIKROSEndscopeController::controlArmsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.q_ref - q) / dt;
    }
}


void EnryuIKROSEndscopeController::controlArmsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.q_old) / dt;
        joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
        info.q_old = q;
    }
}

void EnryuIKROSEndscopeController::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestJoystickState = msg;    
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(EnryuIKROSEndscopeController)
