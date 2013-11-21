#include <ikfast_pr2/ik_interface.h>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <stdio.h>

using namespace boost;

int main(int argc, char** argv){
    IKFastPR2 ik_solver;
    ObjectPose obj;
    obj.x = lexical_cast<double>(argv[1]);
    obj.y = lexical_cast<double>(argv[2]);
    obj.z = lexical_cast<double>(argv[3]);
    obj.roll = lexical_cast<double>(argv[4]);
    obj.pitch = lexical_cast<double>(argv[5]);
    obj.yaw = lexical_cast<double>(argv[6]);

    std::vector<double> r_arm;
    r_arm = ik_solver.getRightArmIKFirstSoln(obj, lexical_cast<double>(argv[7]));
    if (r_arm.size()){
    printf("ik angles\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n", 
           r_arm[0],
           r_arm[1],
           r_arm[2],
           r_arm[3],
           r_arm[4],
           r_arm[5],
           r_arm[6]);
    }

    return 0;
}
