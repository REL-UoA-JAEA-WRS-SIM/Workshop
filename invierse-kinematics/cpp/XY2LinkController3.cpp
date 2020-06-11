/*
	XY2LinkController3.cpp
		Solve inverse kinematics in control() by numerical Jacobain
		Author: Keitaro Naruse
		Date: 2020-06-10
*/

#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <cnoid/SimpleController>

using namespace cnoid;

class XY2LinkController3 : public SimpleController
{
private:
	//  Pointers to link structure for 2link robot arm
	//  2�̃����N�̂��߂̃����N�\���̂ւ̃|�C���^
    Link* joint[2];
    
    //  Reference angle of each of the six joints
    //  2�̊֐߂̂��ꂼ��̖ڕW�p�x
    double q_ref[2];

    //  Joint angles of each of the six joints at the previous time step
    //  2�̊֐߂̂��ꂼ��̕K�O�̎����ł̊֐ߊp�x
    double q_prev[2];
    //  Delta t
    //  ���Ԃ̍��ݕ�
    double dt;

    //  Goal hand position
    //  ���̖ڕW�ʒu
    double p_ref[2];

    /*
        foward_kinematics()
            Input:	q, Joint angle vector = {q1, q2}
            Output:	p, hand position vector = {x, y}
    */
    Eigen::Vector2d forward_kinematics(const Eigen::Vector2d& q)
    {
        //  Link parameters
        //  �����N�p�����[�^
        const double L1 = 0.1;
        const double L2 = 0.1;
        //	Hand position vector
        //  ���̈ʒu�x�N�g��
        Eigen::Vector2d	p;

        //  x component
        //  x���W�̌v�Z
        p(0) = L1 * cos(q(0)) + L2 * cos(q(0) + q(1));
        //  y component        
        //  y���W�̌v�Z
        p(1) = L1 * sin(q(0)) + L2 * sin(q(0) + q(1));

        //  Returns hand position vector
        //  ���̈ʒu�x�N�g����߂�
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
        const double L1 = 0.1;
        const double L2 = 0.1;
        const double dq = 1E-9;
        Eigen::Matrix2d	j;
        Eigen::Vector2d dq1(dq, 0), dq2(0, dq);
        Eigen::Vector2d p, pdp1, pdp2;

        //	forward kinematics of the original angle vector q
        //	����q�ɂ����̈ʒu�x�N�g��
        p = forward_kinematics(q);
        //	forward kinematics of the angle vector q+dq1 (q1 is increased a small amount) 
        //	q+dq1 (q1���������ʑ���)�ɂ����̈ʒu�x�N�g��
        pdp1 = forward_kinematics(q + dq1);
        //	forward kinematics of the angle vector q+dq2 (q2 is increased a small amount) 
        //	q+dq2 (q2���������ʑ���)�ɂ����̈ʒu�x�N�g��
        pdp2 = forward_kinematics(q + dq2);

        //	dx/dt = ((x+dx)-x)/((q+dq1) - q1) 
        //	dy/dt = ((y+dy)-y)/((q+dq1) - q1) 
        j.col(0) = (pdp1 - p) / dq;
        //	dx/dt = ((x+dx)-x)/((q+dq2) - q2) 
        //	dy/dt = ((y+dy)-y)/((q+dq2) - q2) 
        j.col(1) = (pdp2 - p) / dq;

        return(j);
    }

    /*
        pseudo_inverse()
            Input:	m, 2*2 matirx
            Output:	pseudo inverse of a given m
    */
    Eigen::Matrix2d pseudo_inverse(const Eigen::Matrix2d& m)
    {
        //	Psuedo inverse matrix
        //	�^���t�s��
        Eigen::Matrix2d	m_psuedo_inverse;
        //	Sinvgular Value Decomposition of a given m
        //	�^����ꂽ�s��̓��ْl����
        Eigen::JacobiSVD< Eigen::Matrix2d >	svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //	Diagonal vector of singular value matrix and inverse singular value matrix
        //	���ْl�s��Ƌt���ْl�s��̂̑Ίp����
        Eigen::Vector2d	sigma, sigma_inverse;
        sigma = svd.singularValues();

        //	Make an inverse singular value matrix from singular value one
        //	���ْl�s��̋t������t���ْl�s������
        for (int i = 0; i < 2; ++i) {
            //	If a diagonal componets is very small, set sigma_inverse as 0
            //	�Ίp�����̒l��������������C�O��^����
            if (1E-9 > fabs(sigma(i))) {
                sigma_inverse(i) = 0.0;
            }
            //	If a diagonal componets is large enoughr, set sigma_inverse as an inverse of it
            //	�Ίp�����̒l���\���傫��������C���ْl�̋t����^����
            else
            {
                sigma_inverse(i) = 1.0 / sigma(i);
            }
        }
        //	Calulate psuedo inverse matrix from singular value decomposition
        m_psuedo_inverse = svd.matrixV() * sigma_inverse.asDiagonal() * svd.matrixU().transpose();

        return(m_psuedo_inverse);
    }


public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        //  Get a joint from a body 
        //  �֐߂��{�f�B���f������擾
        joint[0] = io -> body() -> link("LINK_1");
        joint[1] = io -> body() -> link("LINK_2");
        
        for(int i = 0; i < 2; i++)	{
            //  Set a joint as torque mode
            //  �֐߂��g���N���[�h�̐ݒ肷��
            joint[i] -> setActuationMode(Link::JOINT_TORQUE);
            //  Set a joint controllable
            //  �֐߂𐧌�\�ɐݒ肷��
            io -> enableIO(joint[i]);
        }
        //  Get a time period of simulation
        //  �V�~�����[�V�����̎��Ԃ̍��ݕ����擾����
        dt = io -> timeStep();
        
        //  Set a reference hand position vector
        //  ���̖ڕW�ʒu��ݒ肷��
        //  as an ordinary array in c++
        p_ref[0] = 0.0;
        p_ref[1] = 0.1;
        return true;
    }
    
    virtual bool control() override
    {
        //  Inverse kinematics solution part: We find q_ref
        //  �t�^���w�v�Z����: q_ref�����߂�

        //	Iteration number
        //	�J��Ԃ���
        const int T = 100;
        //	Time idenx
        //	���Ԃ̓Y����
        int	t;
        //	Coefficient in inverse kinematics solution
        //	�t�^���w�v�Z�̂��߂̌W��
        double	a = 0.1;

        //  Reference hand position vector 
        //  ���̖ڕW�ʒu
        Eigen::Vector2d pg;
        //  Current joint angle vector: {q1, q2} [rad]
        //  ���݂̊֐ߊp�x�x�N�g��
        Eigen::Vector2d	q;
        //  Current Hand position vector: {x, y} [m]
        //  ���݂̎��̈ʒu�x�N�g��
        Eigen::Vector2d	p;
        //  Jacobain
        //  ���R�r�A��
        Eigen::Matrix2d j, pinv;

        //  as a vector in Eigen
        pg(0) = p_ref[0];
        pg(1) = p_ref[1];

        //	Set a current joint angle to a vector for inverse kinematic ssolution
        //	���݂̃{�f�B���f���̊֐ߊp�x���t�^���w�v�Z�p�̊֐ߊp�x�x�N�g���ɗ^����
        q(0) = joint[0]->q(); // input
        q(1) = joint[1]->q(); // input


        //	Iterated calculation for innverse kinematics
        //	�t�^���w�̂��߂̌J��Ԃ��v�Z
        for (t = 0; t < T; ++t) {
            //	forward kinematics
            //	���^���w
            p = forward_kinematics(q);
            //	Find Jacobian
            //���R�r�A��
            j = jacobian(q);
            //	Update joint angles
            //	�֐ߊp�x�̍X�V

            //	Simple inverse matrix does not work in a singular pose
            //	�P���ȋt�s�񂾂Ɠ��َp���̂Ƃ��ɋt�s�񂪒�܂�Ȃ�
            //	q += a * j.inverse() * (pg - p);

            //	Instead, I use psuedo inverse
            //	����ɋ^���t�s���p����
            q += a * pseudo_inverse(j) * (pg - p);
        }

        //  Set a calculated joint angle to reference joint angle
        //  �v�Z���ꂽ�֐ߊp�x���Q�Ɗ֐ߊp�x�Ƃ���
        q_ref[0] = fmod(q(0), 2 * M_PI);
        q_ref[1] = fmod(q(1), 2 * M_PI);

        //  Control part part: We find q_ref
        //  Control a joint to a reference angle
    
        //  PD gains
        //  PD����̂��߂̃Q�C��
        static const double P[2] = { 10.0, 10.0 };
        static const double D[2] = { 5.0,  5.0 };

        //  Joint angle
        //  �֐ߊp�x
        double q_curr[2];
        //  Joint angular velocity
        //  �֐ߊp���x
        double dq[2];
        //  Reference joint angular velocity
        //  �Q�Ɗ֐ߊp���x
        double dq_ref[2] = {0.0, 0.0};

        for (int i = 0; i < 2; i++) {
            //  Get a joint angle from a body model
            //  �{�f�B���f������֐ߊp�x���擾����
            q_curr[i] = joint[i] -> q(); // input
            //  Calculate a joint angular velocity
            //  �P�����O�̊֐ߊp�x���g���āC�֐ߊp���x���v�Z����
            dq[i] = (q_curr[i] - q_prev[i]) / dt;
            //  Determine a joint torque 
            //  PD����Ŋ֐߃g���N�����肷��
            joint[i]->u() = P[i] * (q_ref[i] - q_curr[i]) + D[i] * (dq_ref[i] - dq[i]); // output

            //  Store a current joint angle as a previous one
            //  ���݂̊֐ߊp�x����O�̂��̂Ƃ��ĕۑ�����
            q_prev[i] = q_curr[i];
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(XY2LinkController3)
