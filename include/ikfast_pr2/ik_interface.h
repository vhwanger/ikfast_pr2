/*
 *
 */
#pragma once
#include <kdl/frames.hpp>
#include <vector>
#include <ikfast_pr2/ikfast_right.h>
#include <ikfast_pr2/ikfast_left.h>

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
        bool ikRightArm(const KDL::Frame& wrist_frame, double free_angle, std::vector<double>* angles);
        bool ikAllSolnRightArm(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);
        bool ikAllSolnLeftArm(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);
        bool ikLeftArm(const KDL::Frame& wrist_frame, double free_angle, std::vector<double>* angles);
        KDL::Frame fkRightArm(const std::vector<double> arm_angles);
        KDL::Frame fkLeftArm(const std::vector<double> arm_angles);
        KDL::Frame OR_offset;
};
