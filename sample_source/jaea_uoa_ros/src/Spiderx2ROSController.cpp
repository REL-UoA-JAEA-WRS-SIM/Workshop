/*
  \author Fumiaki Abe
*/
#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

const double STICK_THRESH = 0.1;

}

class Spiderx2ROSController : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;
    Body* body;
    double dt;
    SharedJoystickPtr sharedjoy;
    int targetMode;

    Link::ActuationMode mainActuationMode;

    vector<Link*> tracks;
    Link::ActuationMode trackActuationMode;
    double trackVelocityRatio;
    double kd_mainTrack;
    double kd_subTrack;
    vector<double> qprev_track;

    enum { L_TRACK, R_TRACK, FL_SUB_TRACK, FR_SUB_TRACK, BL_SUB_TRACK, BR_SUB_TRACK, NUM_TRACKS };

    struct JointInfo {
        Link* joint;
        double qref;
        double qold;
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

    enum { SPACER_RF, SPACER_LF, SPACER_RR, SPACER_LR, NUM_FLIPPERS };


    sensor_msgs::Joy joystick;
    bool controlFlg;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeTracks(SimpleControllerIO* io);
    bool initializeTracks(SimpleControllerIO* io, vector<string>& names);
    bool initializeFlipperJoints(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs);
    virtual bool control() override;
    void controlTracks();
    void setTrackTorque(int id, double dq_target, double kd);
    void updateFlipperTargetPositions();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
    void joystickCallback(const sensor_msgs::Joy& msg);
};


bool Spiderx2ROSController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    io->os() << "The actuation mode of " << io->controllerName() << " is ";
    string option = io->optionString();
    if(option == "velocity"){
        mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
        io->os() << "JOINT_VELOCITY";
    } else if(option  == "position"){
        mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
        io->os() << "JOINT_DISPLACEMENT";
    } else {
        mainActuationMode = Link::ActuationMode::JOINT_EFFORT;
        io->os() << "JOINT_EFFORT";
    }
    io->os() << "." << endl;

    if(!initializeTracks(io)){
        return false;
    }

    if(!initializeFlipperJoints(io)){
        return false;
    }

    // joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    // targetMode = joystick->addMode();

    // sharedjoy = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    // targetMode = sharedjoy->addMode();
   

    joystickSubscriber = node.subscribe("joy", 1, &Spiderx2ROSController::joystickCallback, this);
    return true;
}
void Spiderx2ROSController::joystickCallback(const sensor_msgs::Joy& msg)
{
    std::lock_guard<std::mutex> lock(joystickMutex);
    latestJoystickState = msg;
}

bool Spiderx2ROSController::initializeTracks(SimpleControllerIO* io)
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
            trackActuationMode = Link::JOINT_VELOCITY;
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


bool Spiderx2ROSController::initializeTracks(SimpleControllerIO* io, vector<string>& names)
{
    for(auto& name : names){
        auto link = body->link(name);
        if(!link){
            return false;
        }
        link->setActuationMode(trackActuationMode);
        io->enableOutput(link);
        tracks.push_back(link);
        qprev_track.push_back(link->q());
    }
    return true;
}


bool Spiderx2ROSController::initializeFlipperJoints(SimpleControllerIO* io)
{
    jointInfos.clear();
    
    vector<JointSpec> specs(NUM_FLIPPERS);

    const double FLIPPER_P_GAIN_TORQUE = 800.0;
    const double FLIPPER_D_GAIN_TORQUE = 1.0;
    const double FLIPPER_P_GAIN_VELOCITY = 100.3;
    
    specs[SPACER_RF] = { "SPACER_RF", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_LF] = { "SPACER_LF", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_RR] = { "SPACER_RR", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[SPACER_LR] = { "SPACER_LR", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };

    return initializeJoints(io, specs);
}


bool Spiderx2ROSController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs)
{
    for(auto& spec : specs){
        auto joint = body->link(spec.name);
        if(!joint){
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
        
        jointInfos.push_back(info);
    }

    return true;
}


bool Spiderx2ROSController::control()
{
    // joystick->updateState(targetMode);
   //sharedjoy->updateState(targetMode);
    {
        std::lock_guard<std::mutex> lock(joystickMutex);
        joystick = latestJoystickState;
        joystick.axes.resize(10, 0.0f);
        joystick.buttons.resize(13, 0);
    }
    if(joystick.buttons[10]){        
        if(!controlFlg){controlFlg=true;}
        else{controlFlg=false;}
    }
    if(targetMode == 0){
        controlTracks();
        updateFlipperTargetPositions();
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


void Spiderx2ROSController::controlTracks()
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


void Spiderx2ROSController::setTrackTorque(int id, double dq_target, double kd)
{
    Link* axis = tracks[id];
    double dq_current = (axis->q() - qprev_track[id]) / dt;
    axis->u() = kd * (dq_target - dq_current);
    qprev_track[id] = axis->q();
}


void Spiderx2ROSController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.2;

    if(joystick.buttons[12]){
        double qa = 0.0;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            qa += jointInfos[i].qref;
        }
        qa /= NUM_FLIPPERS;
        double dqmax = dt * 0.5;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            double dq = qa - jointInfos[i].qref;
            if(dq > dqmax){
                dq = dqmax;
            } else if(dq < -dqmax){
                dq = -dqmax;
            }
            jointInfos[i].qref += dq;
        }
    } else {
        double pos = joystick.axes[3];
        double dq = dt * FLIPPER_GAIN * pos;
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
            jointInfos[SPACER_RF].qref -= dq;
            jointInfos[SPACER_LF].qref -= dq;
            jointInfos[SPACER_RR].qref += dq;
            jointInfos[SPACER_LR].qref += dq;
        } else {
            if(FL){
                jointInfos[SPACER_LF].qref -= dq;
            }
            if(FR){
                jointInfos[SPACER_RF].qref -= dq;
            }
            if(BL){
                jointInfos[SPACER_LR].qref += dq;
            }
            if(BR){
                jointInfos[SPACER_RR].qref += dq;
            }
        }
    }
}


void Spiderx2ROSController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / dt;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
}


void Spiderx2ROSController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        joint->dq_target() = info.kp * (info.qref - joint->q()) / dt;
    }
}


void Spiderx2ROSController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.qref;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Spiderx2ROSController)