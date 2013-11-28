#include <ikfast_pr2/ik_interface.h>
#include <angles/angles.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

using namespace KDL;
using namespace std;
using namespace angles;
using namespace ikfast;

#define ROT_DATA_SIZE 9

// Hard Joint Limits
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

bool IKFastPR2::ikAllSolnRightArm(const KDL::Frame& wrist_frame, double free_angle,
                          std::vector<std::vector<double> >* soln_list){
    Frame OR_tool_frame = wrist_frame*OR_offset.Inverse();
    //double roll, pitch, yaw;
    //OR_tool_frame.M.GetRPY(roll, pitch, yaw);
    //printf("ikAllSoln frame: %f %f %f (%f %f %f)",
    //        OR_tool_frame.p.x(), OR_tool_frame.p.y(),
    //        OR_tool_frame.p.z(), roll, pitch, yaw);
    
    ik_pr2_rightarm::IkReal eerot[ROT_DATA_SIZE], eetrans[3];
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


    IkSolutionList<ik_pr2_rightarm::IkReal> solutions;
    std::vector<ik_pr2_rightarm::IkReal> vfree(ik_pr2_rightarm::GetNumFreeParameters(), free_angle);
    struct timeval tv_b;
    struct timeval tv_a;
    gettimeofday(&tv_b, NULL);
    long unsigned int before = tv_b.tv_usec + (tv_b.tv_sec * 1000000);

    bool ik_success = ik_pr2_rightarm::ComputeIk(eetrans, eerot, &vfree[0], 
                                              solutions);
    gettimeofday(&tv_a, NULL);
    long unsigned int after = tv_a.tv_usec + (tv_a.tv_sec * 1000000);
    long unsigned int ikfast_time = after - before;
    printf("\ncompute IK is %lu\n", ikfast_time);
    if (!ik_success){
        return false;
    }

    std::vector<ik_pr2_rightarm::IkReal> solvalues(ik_pr2_rightarm::GetNumJoints());
    for(size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        std::vector<double> soln;
        const IkSolutionBase<ik_pr2_rightarm::IkReal>& sol = solutions.GetSolution(i);
        std::vector<ik_pr2_rightarm::IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            soln.push_back(solvalues[j]);
        }
        // Hard limits
#ifdef USE_HARD_JOINT_LIMITS
        if (soln[0] > -2.28540394 && soln[0] < 0.71460237 &&
            soln[1] > -0.52360052 && soln[1] < 1.39630005 &&
            soln[2] > -3.90000803 && soln[2] < 0.79999959 &&
            soln[3] > -2.32130536 && soln[3] < 0 &&
            soln[5] > -2.1800035 && soln[5] < 0){
            soln_list->push_back(soln);
        }
#endif

        // Soft joint limits - this causes ik to fail a lot more, but i'll leave
        // it here just cause.
#ifdef USE_SOFT_JOINT_LIMITS
        if (soln[0] > -2.1353981634 && soln[0] < 0.564601836603 &&
            soln[1] > -.3536 && soln[1] < 1.2963 &&
            soln[2] > -3.75 && soln[2] < .65 &&
            soln[3] > -2.1213 && soln[3] < -.15 &&
            soln[5] > -2 && soln[5] < -.1){
            soln_list->push_back(soln);
        }
#endif
    }
    if (!soln_list->size()){
        return false;
    }
    return true;
}

bool IKFastPR2::ikRightArm(const Frame& wrist_frame, 
                           double free_angle, 
                           vector<double>* angles,
                           bool search_free_angle/*=false*/){
    std::vector<std::vector<double> > all_soln;
    if (ikAllSolnRightArm(wrist_frame, free_angle, &all_soln)){
        *angles = all_soln[0];
        return true;
    // if we want to search over all free angles, let's check angles closest to
    // the given free angle first
    } else if (search_free_angle){
        for (int del_angle=1; del_angle < 180; del_angle++){
            double next_angle = angles::normalize_angle(free_angle + (M_PI/180)*del_angle);
            if (ikAllSolnRightArm(wrist_frame, next_angle, &all_soln)){
                *angles = all_soln[0];
                return true;
            }
            next_angle = angles::normalize_angle(free_angle - (M_PI/180)*del_angle);
            if (ikAllSolnRightArm(wrist_frame, next_angle, &all_soln)){
                *angles = all_soln[0];
                return true;
            }
        }
    }
    return false;
}

KDL::Frame IKFastPR2::fkRightArm(const vector<double> arm_angles){
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




//(u'l_shoulder_pan_joint', (array([-0.71460237]), array([ 2.28540394]))),
//(u'l_shoulder_lift_joint', (array([-0.52360052]), array([ 1.39630005]))),
//(u'l_upper_arm_roll_joint', (array([-0.79999959]), array([ 3.90000803]))),
//(u'l_elbow_flex_joint', (array([-2.32130536]), array([ 0.]))),
//(u'l_forearm_roll_joint', (array([-10000.]), array([ 10000.]))),
//(u'l_wrist_flex_joint', (array([-2.1800035]), array([ 0.]))),
//(u'l_wrist_roll_joint', (array([-10000.]), array([ 10000.]))),


bool IKFastPR2::ikAllSolnLeftArm(const KDL::Frame& wrist_frame, 
                                 double free_angle,
                                 std::vector<std::vector<double> >* soln_list){
    Frame OR_tool_frame = wrist_frame*OR_offset.Inverse();
    //double roll, pitch, yaw;
    //OR_tool_frame.M.GetRPY(roll, pitch, yaw);
    //printf("ikAllSoln frame: %f %f %f (%f %f %f)",
    //        OR_tool_frame.p.x(), OR_tool_frame.p.y(),
    //        OR_tool_frame.p.z(), roll, pitch, yaw);
    
    ik_pr2_leftarm::IkReal eerot[ROT_DATA_SIZE], eetrans[3];
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


    IkSolutionList<ik_pr2_leftarm::IkReal> solutions;
    std::vector<ik_pr2_leftarm::IkReal> vfree(ik_pr2_leftarm::GetNumFreeParameters(), free_angle);
    bool ik_success = ik_pr2_leftarm::ComputeIk(eetrans, eerot, &vfree[0], 
                                              solutions);
    if (!ik_success){
        return false;
    }

    std::vector<ik_pr2_leftarm::IkReal> solvalues(ik_pr2_leftarm::GetNumJoints());
    for(size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        std::vector<double> soln;
        const IkSolutionBase<ik_pr2_leftarm::IkReal>& sol = solutions.GetSolution(i);
        std::vector<ik_pr2_leftarm::IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            soln.push_back(solvalues[j]);
        }
        // Hard limits
#ifdef USE_HARD_JOINT_LIMITS
        if (soln[0] > -0.71460237 && soln[0] < 2.28540394 &&
            soln[1] > -0.52360052 && soln[1] < 1.39630005 &&
            soln[2] > -0.79999959 && soln[2] < 3.90000803 &&
            soln[3] > -2.32130536 && soln[3] < 0 &&
            soln[5] > -2.1800035 && soln[5] < 0){
            soln_list->push_back(soln);
        }
#endif

#ifdef USE_SOFT_JOINT_LIMITS
        // Soft joint limits - this causes ik to fail a lot more, but i'll leave
        // it here just cause.
        if (soln[0] > -0.564601836603 && soln[0] < 2.1353981634 &&
            soln[1] > -.3536 && soln[1] < 1.2963 &&
            soln[2] > -0.65 && soln[2] < 3.75 &&
            soln[3] > -2.1213 && soln[3] < -.15 &&
            soln[5] > -2 && soln[5] < -.1){
            soln_list->push_back(soln);
        }
#endif
    }
    if (!soln_list->size()){
        return false;
    }
    return true;
}

bool IKFastPR2::ikLeftArm(const Frame& wrist_frame, double free_angle, 
                          vector<double>* angles, 
                          bool search_free_angle/*=false*/){
    std::vector<std::vector<double> > all_soln;
    if (ikAllSolnLeftArm(wrist_frame, free_angle, &all_soln)){
        *angles = all_soln[0];
        return true;
    // if we want to search over all free angles, let's check angles closest to
    // the given free angle first
    } else if (search_free_angle){
        for (int del_angle=1; del_angle < 180; del_angle++){
            double next_angle = angles::normalize_angle(free_angle + (M_PI/180)*del_angle);
            if (ikAllSolnLeftArm(wrist_frame, next_angle, &all_soln)){
                *angles = all_soln[0];
                return true;
            }
            next_angle = angles::normalize_angle(free_angle - (M_PI/180)*del_angle);
            if (ikAllSolnLeftArm(wrist_frame, next_angle, &all_soln)){
                *angles = all_soln[0];
                return true;
            }
        }
    }
    return false;
}

KDL::Frame IKFastPR2::fkLeftArm(const vector<double> arm_angles){
    vector<ik_pr2_leftarm::IkReal> IkReal_angles(arm_angles.begin(), arm_angles.end());
    ik_pr2_leftarm::IkReal eetrans[3], eerot[ROT_DATA_SIZE];
    ik_pr2_leftarm::ComputeFk(&IkReal_angles[0], eetrans, eerot);

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
