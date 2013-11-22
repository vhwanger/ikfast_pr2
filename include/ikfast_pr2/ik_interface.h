/*
 *
 */
#pragma once
#include <kdl/frames.hpp>
#include <vector>
#include <ikfast_pr2/ikfast.h>

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
        bool ik(const KDL::Frame& wrist_frame, double free_angle, std::vector<double>* angles);
        bool ikAllSoln(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);
        KDL::Frame getKDLObjectState(const std::vector<double> arm_angles);
        KDL::Frame OR_offset;
};
