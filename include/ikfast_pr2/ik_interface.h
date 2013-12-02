#pragma once
#include <kdl/frames.hpp>
#include <vector>
#include <ikfast_pr2/ikfast_right.h>
#include <ikfast_pr2/ikfast_left.h>

class IKFastPR2 {
    public:
        IKFastPR2();
        bool ikRightArm(const KDL::Frame& wrist_frame, 
                        double free_angle, 
                        std::vector<double>* angles,
                        bool search_free_angle=false);
        bool ikLeftArm(const KDL::Frame& wrist_frame, 
                       double free_angle, 
                       std::vector<double>* angles,
                        bool search_free_angle=false);

        KDL::Frame fkRightArm(const std::vector<double>& arm_angles);
        KDL::Frame fkLeftArm(const std::vector<double>& arm_angles);

        bool ikAllSolnRightArm(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);
        bool ikAllSolnLeftArm(const KDL::Frame&, double free_angle, 
                                  std::vector<std::vector<double> >* all_soln);

    private:
        // this is the offset to go from the openrave FK frame to
        // wrist_roll_link
        KDL::Frame OR_offset;
};
