/**
   HobbyDrone Controller
   @author Kenta Suzuki
*/

#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>
#include <cnoid/RateGyroSensor>
#include <cnoid/RotorDevice>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace Multicopter;

class HobbyDroneVer2InputController : public SimpleController
{
public:
    ros::NodeHandle node;
    ros::Subscriber joystickSubscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;

    BodyPtr ioBody;
    Link* turretJoint[2];
    DeviceList<RotorDevice> rotors;
    RateGyroSensor* gyroSensor;
    double qref[2];
    double qprev[2];
    double dt;

    Vector4 zref;
    Vector4 zprev;
    Vector4 dzref;
    Vector4 dzprev;

    Vector2 xyref;
    Vector2 xyprev;
    Vector2 dxyref;
    Vector2 dxyprev;

    bool on;
    bool manualMode;
    bool mode1;

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();
        rotors = io->body()->devices();
        gyroSensor = ioBody->findDevice<RateGyroSensor>("GyroSensor");
        on = false;
        manualMode = false;
        mode1 = true;

        for(auto opt : io->options()){
            if(opt == "manual") {
                manualMode = true;
            }
            if(opt == "mode2") {
                mode1 = false;
            }
        }

        io->enableInput(ioBody->rootLink(), LINK_POSITION);
        io->enableInput(gyroSensor);

        turretJoint[0] = ioBody->link("TURRET_Y");
        turretJoint[1] = ioBody->link("TURRET_P");
        for(int i = 0; i < 2; i++) {
            Link* joint = turretJoint[i];
            if(!joint) {
                return false;
            }
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        for(size_t i = 0; i < rotors.size(); i++) {
            RotorDevice* rotor = rotors[i];
            io->enableInput(rotor);
        }

        zref = zprev = getZRPY();
        dzref = dzprev = Vector4::Zero();
        xyref = xyprev = getXY();
        dxyref = dxyprev = Vector2::Zero();

        joystickSubscriber = node.subscribe("joy", 1, &HobbyDroneVer2InputController::joystickCallback, this);

        return true;
    }

    void joystickCallback(const sensor_msgs::Joy& msg)
    {
        std::lock_guard<std::mutex> lock(joystickMutex);
        latestJoystickState = msg;
    }

    virtual bool control() override
    {
        sensor_msgs::Joy joystick;
        {
            std::lock_guard<std::mutex> lock(joystickMutex);
            joystick = latestJoystickState;
            joystick.axes.resize(10, 0.0f);
            joystick.buttons.resize(10, 0);
        }

        static bool pprev = false;
        bool p = joystick.buttons[Joystick::A_BUTTON];
        if(p && !pprev) {
            on = !on;
        }
        pprev = p;

        static bool mprev = false;
        bool m = joystick.buttons[Joystick::B_BUTTON];
        if(m && !mprev) {
            mode1 = !mode1;
        }
        mprev = m;

        static const int mode1ID[] = {
            Joystick::R_STICK_V_AXIS,
            Joystick::R_STICK_H_AXIS,
            Joystick::L_STICK_V_AXIS,
            Joystick::L_STICK_H_AXIS
        };

        static const int mode2ID[] = {
            Joystick::L_STICK_V_AXIS,
            Joystick::R_STICK_H_AXIS,
            Joystick::R_STICK_V_AXIS,
            Joystick::L_STICK_H_AXIS
        };

        int axisID[4] = { 0 };
        if(mode1) {
            for(int i = 0; i < 4; i++) {
                axisID[i] = mode1ID[i];
            }
        }
        else {
            for(int i = 0; i < 4; i++) {
                axisID[i] = mode2ID[i];
            }
        }

        static const double P[] = { 10.0, 1.0, 1.0, 0.01 };
        static const double D[] = { 5.0, 1.0, 1.0, 0.002 };
        static const double X[] = { -0.001, 1.0, -1.0, -1.0 };

        static const double KP[] = { 1.0, 1.0 };
        static const double KD[] = { 1.0, 1.0 };
        static const double KX[] = { -1.0, -1.0 };

        static const double sign[4][4] = {
            { 1.0, -1.0, -1.0, -1.0 },
            { 1.0, 1.0, -1.0, 1.0 },
            { 1.0, 1.0, 1.0, -1.0 },
            { 1.0, -1.0, 1.0, 1.0 }
        };

        static const double dir[] = { -1.0, 1.0, -1.0, 1.0 };

        Vector4 f = Vector4::Zero();
        Vector4 z = getZRPY();
        Vector4 dz = (z - zprev) / dt;

        if(gyroSensor) {
            Vector3 w = ioBody->rootLink()->R() * gyroSensor->w();
            dz[3] = w[2];
        }

        Vector4 ddz = (dz - dzprev) / dt;

        Vector2 xy = getXY();
        Vector2 dxy = (xy - xyprev) / dt;
        Vector2 ddxy = (dxy - dxyprev) / dt;
        Vector2 dxy_local = Eigen::Rotation2Dd(-z[3]) * dxy;
        Vector2 ddxy_local = Eigen::Rotation2Dd(-z[3]) * ddxy;

        double cc = cos(z[1]) * cos(z[2]);
        double gfcoef = ioBody->mass() * 9.80665 / 4.0 / cc ;

        if(!on) {
            zref[0] = 0.0;
            dzref[0] = 0.0;
        }

        for(int i = 0; i < 4; i++) {
            double pos = joystick.axes[axisID[i]];
            if(fabs(pos) < 0.20) {
                pos = 0.0;
            }

            if(i == 3) {
                dzref[i] = X[i] * pos;
                f[i] = P[i] * (dzref[i] - dz[i]) + D[i] * (0.0 - ddz[i]);
            }
            else {
                if(i == 0) {
                    zref[i] += X[i] * pos;
                }
                else {
                    if(manualMode) {
                        zref[i] = X[i] * pos;
                    }
                    else {
                        int j = i - 1;
                        dxyref[j] = KX[j] * pos;
                        zref[i] = KP[j] * (dxyref[j] - dxy_local[1 - j])
                                + KD[j] * (0.0 - ddxy_local[1 - j]);
                    }
                }
                if(i == 1) {
                    zref[i] *= -1.0;
                }
                f[i] = P[i] * (zref[i] - z[i]) + D[i] * (0.0 - dz[i]);
            }
        }

        zprev = z;
        dzprev = dz;
        xyprev = xy;
        dxyprev = dxy;

        for(size_t i = 0; i < rotors.size(); i++) {
            RotorDevice* rotor = rotors[i];
            double force = 0.0;
            if(on) {
                force += gfcoef;
                force += sign[i][0] * f[0];
                force += sign[i][1] * f[1];
                force += sign[i][2] * f[2];
                force += sign[i][3] * f[3];
            }
            rotor->setValue(force);
            rotor->setTorque(dir[i] * force);
            rotor->notifyStateChange();
        }

        static const double CP = 0.02;
        static const double CD = 0.001;

        for(int i=0; i < 2; i++) {
            Link* joint = turretJoint[i];
            double pos = joystick.axes[
                i == 0 ? Joystick::DIRECTIONAL_PAD_H_AXIS : Joystick::DIRECTIONAL_PAD_V_AXIS] * 0.3;
            if(fabs(pos) < 0.15) {
                pos = 0.0;
            }

            double q = joint->q();
            double dq = (q - qprev[i]) / dt;
            double dqref = 0.0;
            double deltaq = 0.002 * pos;
            qref[i] += deltaq;
            dqref = deltaq / dt;
            joint->u() = CP * (qref[i] - q) + CD * (dqref - dq);
            qprev[i] = q;
        }

        return true;
    }

    virtual void stop() override
    {
        joystickSubscriber.shutdown();
    }

    Vector4 getZRPY()
    {
        auto T = ioBody->rootLink()->position();
        double z = T.translation().z();
        Vector3 rpy = rpyFromRot(T.rotation());
        return Vector4(z, rpy[0], rpy[1], rpy[2]);
    }
    Vector2 getXY()
    {
        auto p = ioBody->rootLink()->translation();
        return Vector2(p.x(), p.y());
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HobbyDroneVer2InputController)
