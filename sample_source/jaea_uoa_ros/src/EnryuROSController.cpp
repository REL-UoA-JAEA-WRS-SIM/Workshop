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

class EnryuROSController : public SimpleController
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
        NUM_JOINTS
    };


public:
    EnryuROSController();
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


    virtual bool control() override;
    void joystickCallback(const sensor_msgs::Joy& msg);

    Link* link(const char* name) { return body->link(name); }
};

EnryuROSController::EnryuROSController()
{
    mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
    trackType = NO_TRACKS;
}


bool EnryuROSController::initialize(SimpleControllerIO* io)
{
    body = io->body();
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
    specs[MFRAME      ] = { "MFRAME",        200000.0,  20000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",         150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",          150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",           100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",    30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",     20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",     500.0,     50.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",     500.0,     50.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",      50000.0,   5000.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",      30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",       20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",         500.0,     50.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",        50000.0,   5000.0,   P_GAIN_VELOCITY };
    } else {
    //                                          P      D      P (vel)
    specs[MFRAME      ] = { "MFRAME",        20000.0,  2000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",         15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",          15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",           10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",    3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",     2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",     50.0,     5.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",     50.0,     5.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",      5000.0,   500.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",      3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",       2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",         50.0,     5.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",        5000.0,   500.0,   P_GAIN_VELOCITY };
    }

    if(!initializeJoints(io, specs, prefix)){
        return false;
    }
   
    initContinuousTracks(io) || initPseudoContinuousTracks(io);
    initJoystickKeyBind();

    joystickSubscriber = nh.subscribe("joy",1, &EnryuROSController::joystickCallback,this);

    return true;
}

bool EnryuROSController::start()
{
    return true;
}


bool EnryuROSController::initContinuousTracks(SimpleControllerIO* io)
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

bool EnryuROSController::initPseudoContinuousTracks(SimpleControllerIO* io)
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


bool EnryuROSController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
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


void EnryuROSController::initJoystickKeyBind()
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
            { link("MNP_SWING"),    STICK,  R_STICK_H_AXIS, -0.6 },
            { link("MANIBOOM"),     STICK,  L_STICK_V_AXIS, -0.6 },
            { link("MANIARM"),      STICK,  R_STICK_V_AXIS,  0.6 },
            { link("MANIELBOW"),    BUTTON, A_BUTTON,        0.6 },
            { link("MANIELBOW"),    BUTTON, Y_BUTTON,       -0.6 },
            { link("YAWJOINT"),     BUTTON, X_BUTTON,        1.0, 1 },
            { link("YAWJOINT"),     BUTTON, B_BUTTON,       -1.0, 1 },
            { link("HANDBASE"),     BUTTON, X_BUTTON,       -1.0, 0 },
            { link("HANDBASE"),     BUTTON, B_BUTTON,        1.0, 0 },
            { link("PUSHROD"),      STICK,  R_TRIGGER_AXIS, -0.04 },
            { link("PUSHROD"),      BUTTON, R_BUTTON,        0.04 },
        }
    };

    operationSetIndex = 0;
}

void EnryuROSController::controlTracks()
{
    trackL->u() = 0.0;
    trackL->dq() = 0.0;
    trackR->u() = 0.0;
    trackR->dq() = 0.0;
    
    const double k1 = 0.4;
    const double k2 = 0.6;

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


bool EnryuROSController::control()
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
            shiftState = 0;
            if(operationSetIndex >= 2){
                operationSetIndex = 0;
                shiftState=0;
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


void EnryuROSController::setTargetArmPositions()
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

void EnryuROSController::controlArms()
{
    setTargetArmPositions();

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


void EnryuROSController::controlArmsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.q_ref;
    }
}


void EnryuROSController::controlArmsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.q_ref - q) / dt;
    }
}


void EnryuROSController::controlArmsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.q_old) / dt;
        joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
        info.q_old = q;
    }
}

void EnryuROSController::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    latestJoystickState = msg;    
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(EnryuROSController)
