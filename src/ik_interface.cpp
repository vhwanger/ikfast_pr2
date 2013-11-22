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
#define NUM_FREE_ANGLES 1

//min_joint_limits:  -0.564602 -0.353600 -0.6500 -2.121300 0 -2.0 0
//max_joint_limits:   2.135398  1.296300  3.7500 -0.15000  0 -0.1 0
//-0.015788 1.300050 -0.972990 -2.167336 0.905603 1.722229 2.325315
//
//
//min -2.1353981634 -0.3536 -3.75 -2.1213 0 -2    0
//max 0.564601836603 1.2963 0.65 -0.15    0 -.1   0
//-0.018939 1.058278 0.637614 -0.151403 -2.409291 -0.529954 1.691271

IKFastPR2::IKFastPR2(){
    KDL::Rotation rot = KDL::Rotation::Quaternion(0, -pow(2,.5)/2, 0, pow(2,.5)/2);
    KDL::Vector v(0, 0, -.18);

    

    OR_offset = KDL::Frame(rot, v);
}

double normalizeFreeAngle(double value){
    double rotation = 2*M_PI;
    while (value < -.65){
        value += rotation;
    }
    while (value > 3.75){
        value -= rotation;
    }
    return value;
}

bool IKFastPR2::ikAllSoln(const KDL::Frame& wrist_frame, double free_angle,
                          std::vector<std::vector<double> >* soln_list){
    //free_angle = angles::normalize_angle(free_angle);
    Frame OR_tool_frame = wrist_frame*OR_offset.Inverse();
    double roll, pitch, yaw;
    OR_tool_frame.M.GetRPY(roll, pitch, yaw);
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
    bool success = ik_pr2_rightarm::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    std::vector<IkReal> solvalues(GetNumJoints());

    for(size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        std::vector<double> soln;

        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j){
            soln.push_back(solvalues[j]);
            //printf("%f ", solvalues[j]);
        }
        //printf("\n");
 //(u'r_shoulder_pan_joint', (array([-2.28540394]), array([ 0.71460237]))),
 //(u'r_shoulder_lift_joint', (array([-0.52360052]), array([ 1.39630005]))),
 //(u'r_upper_arm_roll_joint', (array([-3.90000803]), array([ 0.79999959]))),
 //(u'r_elbow_flex_joint', (array([-2.32130536]), array([ 0.]))),
 //(u'r_forearm_roll_joint', (array([-10000.]), array([ 10000.]))),
 //(u'r_wrist_flex_joint', (array([-2.1800035]), array([ 0.]))),
 //(u'r_wrist_roll_joint', (array([-10000.]), array([ 10000.]))),
        if (soln[0] > -2.28540394 && soln[0] < 0.71460237 &&
            soln[1] > -0.52360052 && soln[1] < 1.39630005 &&
            soln[2] > -3.90000803 && soln[2] < 0.79999959 &&
            soln[3] > -2.32130536 && soln[3] < 0 &&
            soln[5] > -2.1800035 && soln[5] < 0){
            soln_list->push_back(soln);
            for( std::size_t j = 0; j < solvalues.size(); ++j){
                //printf("%f ", solvalues[j]);
            }
        }
        //if (soln[0] > -2.1353981634 && soln[0] < 0.564601836603 &&
        //    soln[1] > -.3536 && soln[1] < 1.2963 &&
        //    soln[2] > -3.75 && soln[2] < .65 &&
        //    soln[3] > -2.1213 && soln[3] < -.15 &&
        //    soln[5] > -2 && soln[5] < -.1){
        //    soln_list->push_back(soln);
        //    for( std::size_t j = 0; j < solvalues.size(); ++j){
        //        printf("%f ", solvalues[j]);
        //    }
        //}
    }
    if (!soln_list->size()){
        return false;
    }
    return true;
}
bool IKFastPR2::ik(const ObjectState& obj_pose, double free_angle, vector<double>* angles){
    KDL::Vector v(obj_pose.x, obj_pose.y, obj_pose.z);
    KDL::Rotation rot = KDL::Rotation::RPY(obj_pose.roll, obj_pose.pitch, obj_pose.yaw);
    KDL::Frame wrist_frame(rot, v);
    std::vector<std::vector<double> > all_soln;
    if (ikAllSoln(wrist_frame, free_angle, &all_soln)){
        *angles = all_soln[0];
        return true;
    } else {
        return false;
    }
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

ObjectState IKFastPR2::getRightArmObjectState(const vector<double> arm_angles){
    KDL::Frame frame = getKDLObjectState(arm_angles);
    ObjectState obj;
    obj.x = frame.p.x();
    obj.y = frame.p.y();
    obj.z = frame.p.z();
    frame.M.GetRPY(obj.roll, obj.pitch, obj.yaw);
    return obj;
}
