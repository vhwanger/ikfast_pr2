#include <ikfast_pr2/ik_interface.h>
#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <stdio.h>

using namespace KDL;
using namespace std;
using namespace angles;
using namespace ikfast;
using namespace ik_pr2_rightarm;

#define ROT_DATA_SIZE 9

//Joint Limits
//
// r_shoulder_pan_joint', -2.28540394, 0.71460237
// r_shoulder_lift_joint', -0.52360052, 1.39630005
// r_upper_arm_roll_joint', -3.90000803, 0.79999959
// r_elbow_flex_joint', -2.32130536, 0.
// r_forearm_roll_joint', -10000., 10000.
// r_wrist_flex_joint', -2.1800035, 0.
// r_wrist_roll_joint', -10000., 10000.

// Openrave's FK and IK actually go between /torso_lift_link and some other
// frame that has the same xyz as /r_gripper_tool_frame, but a different
// orientation. This offset moves everything over to r_wrist_roll_link.
IKFastPR2::IKFastPR2(){
    KDL::Rotation rot = KDL::Rotation::Quaternion(0, -pow(2,.5)/2, 0, pow(2,.5)/2);
    KDL::Vector v(0, 0, -.18);
    OR_offset = KDL::Frame(rot, v);
}

bool IKFastPR2::ikAllSoln(const KDL::Frame& wrist_frame, double free_angle,
                          std::vector<std::vector<double> >* soln_list){
    Frame OR_tool_frame = wrist_frame*OR_offset.Inverse();
    //double roll, pitch, yaw;
    //OR_tool_frame.M.GetRPY(roll, pitch, yaw);
    //printf("ikAllSoln frame: %f %f %f (%f %f %f)",
    //        OR_tool_frame.p.x(), OR_tool_frame.p.y(),
    //        OR_tool_frame.p.z(), roll, pitch, yaw);
    
    IkReal eerot[ROT_DATA_SIZE], eetrans[3];
    eetrans[0] = OR_tool_frame.p.x();
    eetrans[1] = OR_tool_frame.p.y();
    eetrans[2] = OR_tool_frame.p.z();

    for (int i=0; i < ROT_DATA_SIZE; i++){
        eerot[i] = OR_tool_frame.M.data[i];
    }

    //printf("\nfree angle: %f\n", free_angle);
    //printf("\n%f %f %f", eetrans[0], eetrans[1], eetrans[2]);
    //printf("\n%f %f %f\n%f %f %f\n%f %f %f\n",
    //        eerot[0],
    //        eerot[1],
    //        eerot[2],
    //        eerot[3],
    //        eerot[4],
    //        eerot[5],
    //        eerot[6],
    //        eerot[7],
    //        eerot[8]);


    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters(), free_angle);
    bool ik_success = ik_pr2_rightarm::ComputeIk(eetrans, eerot, &vfree[0], 
                                              solutions);
    if (!ik_success){
        return false;
    }

    std::vector<IkReal> solvalues(GetNumJoints());
    for(size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        std::vector<double> soln;
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            soln.push_back(solvalues[j]);
        }
        // Hard limits
        //if (soln[0] > -2.28540394 && soln[0] < 0.71460237 &&
        //    soln[1] > -0.52360052 && soln[1] < 1.39630005 &&
        //    soln[2] > -3.90000803 && soln[2] < 0.79999959 &&
        //    soln[3] > -2.32130536 && soln[3] < 0 &&
        //    soln[5] > -2.1800035 && soln[5] < 0){
        //    soln_list->push_back(soln);
        //}

        // Soft joint limits - this causes ik to fail a lot more, but i'll leave
        // it here just cause.
        //
        if (soln[0] > -2.1353981634 && soln[0] < 0.564601836603 &&
            soln[1] > -.3536 && soln[1] < 1.2963 &&
            soln[2] > -3.75 && soln[2] < .65 &&
            soln[3] > -2.1213 && soln[3] < -.15 &&
            soln[5] > -2 && soln[5] < -.1){
            soln_list->push_back(soln);
        }
    }
    if (!soln_list->size()){
        return false;
    }
    return true;
}

bool IKFastPR2::ik(const Frame& wrist_frame, double free_angle, vector<double>* angles){
    std::vector<std::vector<double> > all_soln;
    if (ikAllSoln(wrist_frame, free_angle, &all_soln)){
        *angles = all_soln[0];
        return true;
    } else {
        return false;
    }
}

KDL::Frame IKFastPR2::getKDLObjectState(const vector<double> arm_angles){
    vector<ik_pr2_rightarm::IkReal> IkReal_angles(arm_angles.begin(), arm_angles.end());
    ik_pr2_rightarm::IkReal eetrans[3], eerot[ROT_DATA_SIZE];
    ik_pr2_rightarm::ComputeFk(&IkReal_angles[0], eetrans, eerot);

    KDL::Vector kdl_v(eetrans[0], eetrans[1], eetrans[2]);
    KDL::Rotation rot(eerot[0], eerot[1], eerot[2],
                      eerot[3], eerot[4], eerot[5],
                      eerot[6], eerot[7], eerot[8]);

    //printf("free ange: %f\n", arm_angles[UPPER_ARM_ROLL]);
    //printf("\n%f %f %f", eetrans[0], eetrans[1], eetrans[2]);
    //printf("\n%f %f %f\n%f %f %f\n%f %f %f\n",
    //        eerot[0],
    //        eerot[1],
    //        eerot[2],
    //        eerot[3],
    //        eerot[4],
    //        eerot[5],
    //        eerot[6],
    //        eerot[7],
    //        eerot[8]);
    KDL::Frame wrist_frame = KDL::Frame(rot, kdl_v)*OR_offset;
    double roll, pitch, yaw;
    wrist_frame.M.GetRPY(roll, pitch, yaw);
    vector<vector<double> > test;
    return wrist_frame;
}
