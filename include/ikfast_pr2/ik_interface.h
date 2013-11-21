#pragma once
#include <kdl/frames.hpp>
#include <vector>
//#include <ikfast_pr2/ik_right.h>
#include <ikfast_pr2/ikfast.h>

struct ObjectState {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum Joints {
    SHOULDER_PAN,
    SHOULDER_LIFT,
    UPPER_ARM_ROLL,
    ELBOW_FLEX,
    FOREARM_ROLL,
    WRIST_FLEX,
    WRIST_ROLL
};

class IKFastPR2 {
    public:
        IKFastPR2();
        // returns the ik solution closest to the given free angle
        bool ik(const ObjectState&, double free_angle, std::vector<double>* angles);
        bool ik(const KDL::Frame& wrist_frame, double free_angle, std::vector<double>* angles);
        bool ikAllSoln(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);
        ObjectState getRightArmObjectState(const std::vector<double>);
        KDL::Frame getKDLObjectState(const std::vector<double> arm_angles);
        KDL::Frame OR_offset;
};
