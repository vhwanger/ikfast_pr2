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

bool IKFastPR2::ikAllSoln(const ObjectState& obj_pose, double free_angle,
                          std::vector<std::vector<double> >* soln_list){
    Rotation rot = Rotation::RPY(obj_pose.roll, obj_pose.pitch, obj_pose.yaw);
    IkReal eerot[ROT_DATA_SIZE], eetrans[3];
    eetrans[0] = obj_pose.x;
    eetrans[1] = obj_pose.y;
    eetrans[2] = obj_pose.z;

    for (int i=0; i < ROT_DATA_SIZE; i++){
        eerot[i] = rot.data[i];
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
            printf("%f ", solvalues[j]);
        }
        printf("\n");
        if (soln[0] > -.564602 && soln[0] < 2.135398 &&
            soln[1] > -.3536 && soln[1] < 1.2963 &&
            soln[2] > -.65 && soln[2] < 3.75 &&
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
bool IKFastPR2::ik(const ObjectState& obj_pose, double free_angle, vector<double>* angles){
    std::vector<std::vector<double> > all_soln;
    if (ikAllSoln(obj_pose, free_angle, &all_soln)){
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
    KDL::Rotation offset(0, 0, 1,
                         0, 1, 0,
                         -1, 0, 0);
    KDL::Rotation new_rot = rot*offset;

    return KDL::Frame(new_rot, kdl_v);
}

ObjectState IKFastPR2::getRightArmObjectState(const vector<double> arm_angles){
    vector<ik_pr2_rightarm::IkReal> IkReal_angles(arm_angles.begin(), arm_angles.end());
    ik_pr2_rightarm::IkReal eetrans[3], eerot[ROT_DATA_SIZE];
    ik_pr2_rightarm::ComputeFk(&IkReal_angles[0], eetrans, eerot);
    
    KDL::Rotation rot(eerot[0], eerot[1], eerot[2],
                      eerot[3], eerot[4], eerot[5],
                      eerot[6], eerot[7], eerot[8]);
    double testr, testp, testy;
    rot.GetRPY(testr, testp, testy);
    //printf("before adjust rpy: %f %f %f %f %f %f\n", eetrans[0], eetrans[1], eetrans[2], testr, testp, testy);

    ObjectState obj_pose;
    obj_pose.x = eetrans[0];
    obj_pose.y = eetrans[1];
    obj_pose.z = eetrans[2];
    rot.GetRPY(obj_pose.roll, obj_pose.pitch, obj_pose.yaw);
    return obj_pose;
}
