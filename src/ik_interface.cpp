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

IKFastPR2::IKFastPR2(){
    KDL::Rotation rot = KDL::Rotation::Quaternion(0, -pow(2,.5)/2, 0, pow(2,.5)/2);
    //KDL::Rotation rot = KDL::Rotation::Quaternion(0, 1, 0, 0);
    KDL::Vector v(0, 0, -.18);
    OR_offset = KDL::Frame(rot, v);
}
bool IKFastPR2::ikAllSoln(const KDL::Frame& wrist_frame, double free_angle,
                          std::vector<std::vector<double> >* soln_list){
    Frame OR_tool_frame = wrist_frame*OR_offset.Inverse();
    
    IkReal eerot[ROT_DATA_SIZE], eetrans[3];
    eetrans[0] = OR_tool_frame.p.x();
    eetrans[1] = OR_tool_frame.p.y();
    eetrans[2] = OR_tool_frame.p.z();

    for (int i=0; i < ROT_DATA_SIZE; i++){
        eerot[i] = OR_tool_frame.M.data[i];
    }
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
        }
        printf("\n");
        if (soln[0] > -.564602 && soln[0] < 2.135398 &&
            soln[1] > -.3536 && soln[1] < 1.2963 &&
            soln[2] > -.65 && soln[2] < 3.75 &&
            soln[3] > -2.1213 && soln[3] < -.15 &&
            soln[5] > -2 && soln[5] < -.1){
            soln_list->push_back(soln);
            for( std::size_t j = 0; j < solvalues.size(); ++j){
                printf("%f ", solvalues[j]);
            }
        }
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

    return KDL::Frame(rot, kdl_v)*OR_offset;
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
